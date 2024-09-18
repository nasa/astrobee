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
// x12  <- A_p
// x13  <- B_p
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NT_8X4_P0_LIB4
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_8x4_p0_lib4)
#endif



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x14, x9, x10

	add		x15, x12, x10 // A_p1

	// prefetch
//	prfm	PLDL1KEEP, [x11, #0]
//	prfm	PLDL1KEEP, [x11, #64]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x14, #0]
//	prfm	PLDL1KEEP, [x9, #64]
//	prfm	PLDL1KEEP, [x14, #64]

//	prfm	PLDL1KEEP, [x12, #0]
//	prfm	PLDL1KEEP, [x14, #0]
//	prfm	PLDL1KEEP, [x12, #64]
//	prfm	PLDL1KEEP, [x14, #64]

	// preload
	ldp		q24, q25, [x11, #(0*8+0*32)]
	ldp		q26, q27, [x11, #(0*8+1*32)]
	ldp		q28, q29, [x11, #(0*8+2*32)]
	ldp		q30, q31, [x11, #(0*8+3*32)]
	ldp		q16, q17, [x9, #(0*8+0*32)]
	ldp		q20, q21, [x14, #(0*8+0*32)]

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// main loop
1:
	
	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q18, q19, [x9, #(0*8+1*32)]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	ldp		q22, q23, [x14, #(0*8+1*32)]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, #128]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	prfm	PLDL1KEEP, [x14, #128]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x9, #192]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
	prfm	PLDL1KEEP, [x14, #192]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	prfm	PLDL1KEEP, [x11, #128]
	fmla	v14.2d, v20.2d, v25.d[1]
	fmla	v15.2d, v21.2d, v25.d[1]
	prfm	PLDL1KEEP, [x11, #192]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldp		q16, q17, [x9, #(0*8+2*32)]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	ldp		q20, q21, [x14, #(0*8+2*32)]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]
	add		x11, x11, #128
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	sub		w8, w8, #4
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	// unroll 2
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldp		q18, q19, [x9, #(0*8+3*32)]
	fmla	v8.2d, v20.2d, v28.d[0]
	fmla	v9.2d, v21.2d, v28.d[0]
	ldp		q22, q23, [x14, #(0*8+3*32)]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v11.2d, v21.2d, v28.d[1]
	add		x9, x9, #128
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	add		x14, x14, #128
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v13.2d, v21.2d, v29.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v29.d[1]
	fmla	v7.2d, v17.2d, v29.d[1]
	ldp		q16, q17, [x9, #(0*8+0*32)]
	fmla	v14.2d, v20.2d, v29.d[1]
	fmla	v15.2d, v21.2d, v29.d[1]
	ldp		q20, q21, [x14, #(0*8+0*32)]

	// unroll 3
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
	fmla	v8.2d, v22.2d, v30.d[0]
	fmla	v9.2d, v23.2d, v30.d[0]
	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v30.d[1]
	fmla	v3.2d, v19.2d, v30.d[1]
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
	ldp		q26, q27, [x11, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v31.d[0]
	fmla	v5.2d, v19.2d, v31.d[0]
	fmla	v12.2d, v22.2d, v31.d[0]
	fmla	v13.2d, v23.2d, v31.d[0]
	ldp		q28, q29, [x11, #(0*8+2*32)]
	fmla	v6.2d, v18.2d, v31.d[1]
	fmla	v7.2d, v19.2d, v31.d[1]
	fmla	v14.2d, v22.2d, v31.d[1]
	fmla	v15.2d, v23.2d, v31.d[1]
	ldp		q30, q31, [x11, #(0*8+3*32)]

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q18, q19, [x9, #(0*8+1*32)]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	ldp		q22, q23, [x14, #(0*8+1*32)]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	prfm	PLDL1KEEP, [x12, #0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	prfm	PLDL1KEEP, [x15, #0]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x12, #64]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
	prfm	PLDL1KEEP, [x15, #64]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	prfm	PLDL1KEEP, [x13, #64]
	fmla	v14.2d, v20.2d, v25.d[1]
	fmla	v15.2d, v21.2d, v25.d[1]
	prfm	PLDL1KEEP, [x13, #64]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldp		q16, q17, [x9, #(0*8+2*32)]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	ldp		q20, q21, [x14, #(0*8+2*32)]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]
	add		x11, x11, #128
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	sub		w8, w8, #4
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	// unroll 2
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldp		q18, q19, [x9, #(0*8+3*32)]
	fmla	v8.2d, v20.2d, v28.d[0]
	fmla	v9.2d, v21.2d, v28.d[0]
	ldp		q22, q23, [x14, #(0*8+3*32)]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v11.2d, v21.2d, v28.d[1]
	add		x9, x9, #128
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	add		x14, x14, #128
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v13.2d, v21.2d, v29.d[0]
	fmla	v6.2d, v16.2d, v29.d[1]
	fmla	v7.2d, v17.2d, v29.d[1]
//	ldp		q16, q17, [x9, #(0*8+0*32)]
	fmla	v14.2d, v20.2d, v29.d[1]
	fmla	v15.2d, v21.2d, v29.d[1]
//	ldp		q20, q21, [x14, #(0*8+0*32)]

	// unroll 3
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
	fmla	v8.2d, v22.2d, v30.d[0]
	fmla	v3.2d, v19.2d, v30.d[1]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v9.2d, v23.2d, v30.d[0]
	fmla	v2.2d, v18.2d, v30.d[1]
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v31.d[0]
	fmla	v5.2d, v19.2d, v31.d[0]
	fmla	v12.2d, v22.2d, v31.d[0]
	fmla	v13.2d, v23.2d, v31.d[0]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
	fmla	v6.2d, v18.2d, v31.d[1]
	fmla	v7.2d, v19.2d, v31.d[1]
	fmla	v14.2d, v22.2d, v31.d[1]
	fmla	v15.2d, v23.2d, v31.d[1]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x9, x9, #32
//	sub		x11, x11, #32
//	sub		x14, x14, #32

3: // clean1-up loop

	// unroll 0
	ld1		{v20.2d, v21.2d}, [x9], #32
	ld1		{v28.2d, v29.2d}, [x11], #32
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	fmla	v6.2d, v20.2d, v29.d[1]
	fmla	v7.2d, v21.2d, v29.d[1]
	ld1		{v22.2d, v23.2d}, [x14], #32
	fmla	v8.2d, v22.2d, v28.d[0]
	fmla	v9.2d, v23.2d, v28.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v22.2d, v28.d[1]
	fmla	v11.2d, v23.2d, v28.d[1]
	cmp		w8, #0
	fmla	v12.2d, v22.2d, v29.d[0]
	fmla	v13.2d, v23.2d, v29.d[0]
	fmla	v14.2d, v22.2d, v29.d[1]
	fmla	v15.2d, v23.2d, v29.d[1]

	bgt		3b

2: // return



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_8x4_p0_lib4)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- sda
// x11  <- B
// x12  <- A_p
// x13  <- B_p
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NT_8X4_PL_LIB4
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_8x4_pl_lib4)
#endif



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x14, x9, x10

	add		x15, x12, x10 // p

	mov		x16, x12
	mov		x17, x15

	// prefetch
//	prfm	PLDL1KEEP, [x11, #0]
//	prfm	PLDL1KEEP, [x11, #64]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x14, #0]
//	prfm	PLDL1KEEP, [x9, #64]
//	prfm	PLDL1KEEP, [x14, #64]

//	prfm	PLDL1KEEP, [x12, #0]
//	prfm	PLDL1KEEP, [x14, #0]
//	prfm	PLDL1KEEP, [x12, #64]
//	prfm	PLDL1KEEP, [x14, #64]

	// preload
	ldp		q24, q25, [x11, #(0*8+0*32)]
	ldp		q26, q27, [x11, #(0*8+1*32)]
	ldp		q28, q29, [x11, #(0*8+2*32)]
	ldp		q30, q31, [x11, #(0*8+3*32)]
	ldp		q16, q17, [x9, #(0*8+0*32)]
	ldp		q20, q21, [x14, #(0*8+0*32)]

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// main loop
1:
	
	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q18, q19, [x9, #(0*8+1*32)]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	ldp		q22, q23, [x14, #(0*8+1*32)]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, #128]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	prfm	PLDL1KEEP, [x14, #128]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x9, #192]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
	prfm	PLDL1KEEP, [x9, #192]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	prfm	PLDL1KEEP, [x11, #128]
	fmla	v14.2d, v20.2d, v25.d[1]
	fmla	v15.2d, v21.2d, v25.d[1]
	prfm	PLDL1KEEP, [x11, #192]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldp		q16, q17, [x9, #(0*8+2*32)]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	ldp		q20, q21, [x14, #(0*8+2*32)]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]
	add		x11, x11, #128
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	sub		w8, w8, #4
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	// unroll 2
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldp		q18, q19, [x9, #(0*8+3*32)]
	fmla	v8.2d, v20.2d, v28.d[0]
	fmla	v9.2d, v21.2d, v28.d[0]
	ldp		q22, q23, [x14, #(0*8+3*32)]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v11.2d, v21.2d, v28.d[1]
	add		x9, x9, #128
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	add		x14, x14, #128
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v13.2d, v21.2d, v29.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v29.d[1]
	fmla	v7.2d, v17.2d, v29.d[1]
	ldp		q16, q17, [x9, #(0*8+0*32)]
	fmla	v14.2d, v20.2d, v29.d[1]
	fmla	v15.2d, v21.2d, v29.d[1]
	ldp		q20, q21, [x14, #(0*8+0*32)]

	// unroll 3
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
	prfm	PLDL2KEEP, [x16, #0]
	fmla	v8.2d, v22.2d, v30.d[0]
	fmla	v9.2d, v23.2d, v30.d[0]
	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v30.d[1]
	fmla	v3.2d, v19.2d, v30.d[1]
	prfm	PLDL2KEEP, [x17, #0]
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
	ldp		q26, q27, [x11, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v31.d[0]
	prfm	PLDL2KEEP, [x16, #64]
	fmla	v5.2d, v19.2d, v31.d[0]
	prfm	PLDL2KEEP, [x17, #64]
	fmla	v12.2d, v22.2d, v31.d[0]
	fmla	v13.2d, v23.2d, v31.d[0]
	ldp		q28, q29, [x11, #(0*8+2*32)]
	fmla	v6.2d, v18.2d, v31.d[1]
	fmla	v7.2d, v19.2d, v31.d[1]
	add		x17, x17, #128
	fmla	v14.2d, v22.2d, v31.d[1]
	add		x16, x16, #128
	fmla	v15.2d, v23.2d, v31.d[1]
	ldp		q30, q31, [x11, #(0*8+3*32)]

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q18, q19, [x9, #(0*8+1*32)]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	ldp		q22, q23, [x14, #(0*8+1*32)]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	prfm	PLDL1KEEP, [x12, #0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	prfm	PLDL1KEEP, [x15, #0]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x12, #64]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
	prfm	PLDL1KEEP, [x15, #64]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	prfm	PLDL1KEEP, [x13, #0]
	fmla	v14.2d, v20.2d, v25.d[1]
	fmla	v15.2d, v21.2d, v25.d[1]
	prfm	PLDL1KEEP, [x13, #64]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldp		q16, q17, [x9, #(0*8+2*32)]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	ldp		q20, q21, [x14, #(0*8+2*32)]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]
	add		x11, x11, #128
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	sub		w8, w8, #4
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	// unroll 2
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldp		q18, q19, [x9, #(0*8+3*32)]
	fmla	v8.2d, v20.2d, v28.d[0]
	fmla	v9.2d, v21.2d, v28.d[0]
	ldp		q22, q23, [x14, #(0*8+3*32)]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v11.2d, v21.2d, v28.d[1]
	add		x9, x9, #128
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	add		x14, x14, #128
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v13.2d, v21.2d, v29.d[0]
	fmla	v6.2d, v16.2d, v29.d[1]
	fmla	v7.2d, v17.2d, v29.d[1]
//	ldp		q16, q17, [x9, #(0*8+0*32)]
	fmla	v14.2d, v20.2d, v29.d[1]
	fmla	v15.2d, v21.2d, v29.d[1]
//	ldp		q20, q21, [x14, #(0*8+0*32)]

	// unroll 3
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
	prfm	PLDL2KEEP, [x16, #0]
	fmla	v8.2d, v22.2d, v30.d[0]
	fmla	v3.2d, v19.2d, v30.d[1]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	prfm	PLDL2KEEP, [x17, #0]
	fmla	v9.2d, v23.2d, v30.d[0]
	fmla	v2.2d, v18.2d, v30.d[1]
	prfm	PLDL2KEEP, [x16, #64]
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
	prfm	PLDL2KEEP, [x17, #64]
	fmla	v4.2d, v18.2d, v31.d[0]
	fmla	v5.2d, v19.2d, v31.d[0]
	add		x16, x16, #128
	fmla	v12.2d, v22.2d, v31.d[0]
	fmla	v13.2d, v23.2d, v31.d[0]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
	add		x17, x17, #128
	fmla	v6.2d, v18.2d, v31.d[1]
	fmla	v7.2d, v19.2d, v31.d[1]
	fmla	v14.2d, v22.2d, v31.d[1]
	fmla	v15.2d, v23.2d, v31.d[1]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x9, x9, #32
//	sub		x11, x11, #32
//	sub		x14, x14, #32

3: // clean1-up loop

	// unroll 0
	ld1		{v20.2d, v21.2d}, [x9], #32
	ld1		{v28.2d, v29.2d}, [x11], #32
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	fmla	v6.2d, v20.2d, v29.d[1]
	fmla	v7.2d, v21.2d, v29.d[1]
	ld1		{v22.2d, v23.2d}, [x14], #32
	fmla	v8.2d, v22.2d, v28.d[0]
	fmla	v9.2d, v23.2d, v28.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v22.2d, v28.d[1]
	fmla	v11.2d, v23.2d, v28.d[1]
	cmp		w8, #0
	fmla	v12.2d, v22.2d, v29.d[0]
	fmla	v13.2d, v23.2d, v29.d[0]
	fmla	v14.2d, v22.2d, v29.d[1]
	fmla	v15.2d, v23.2d, v29.d[1]

	bgt		3b

2: // return



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_8x4_pl_lib4)
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
	.macro INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_8x4_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x9, x10

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// main loop
1:
	
	// load 0 & 1
	ldp		q16, q17, [x9], #32
	ldp		q20, q21, [x13], #32
	ldp		q24, q25, [x11]
	add		x11, x11, x12
	ldp		q18, q19, [x9], #32
	ldp		q22, q23, [x13], #32
	ldp		q26, q27, [x11]
	add		x11, x11, x12

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
	fmla	v14.2d, v20.2d, v25.d[1]
	fmla	v15.2d, v21.2d, v25.d[1]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	// load 2 & 3
	ldp		q16, q17, [x9], #32
	ldp		q20, q21, [x13], #32
	ldp		q24, q25, [x11]
	add		x11, x11, x12
	ldp		q18, q19, [x9], #32
	ldp		q22, q23, [x13], #32
	ldp		q26, q27, [x11]
	add		x11, x11, x12

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
	fmla	v14.2d, v20.2d, v25.d[1]
	fmla	v15.2d, v21.2d, v25.d[1]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldp		q16, q17, [x9], #32
	ldp		q20, q21, [x13], #32
	ldp		q24, q25, [x11]
	add		x11, x11, x12
	ldp		q18, q19, [x9], #32
	ldp		q22, q23, [x13], #32
	ldp		q26, q27, [x11]
	add		x11, x11, x12

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
	fmla	v14.2d, v20.2d, v25.d[1]
	fmla	v15.2d, v21.2d, v25.d[1]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	// load 2 & 3
	ldp		q16, q17, [x9], #32
	ldp		q20, q21, [x13], #32
	ldp		q24, q25, [x11]
	add		x11, x11, x12
	ldp		q18, q19, [x9], #32
	ldp		q22, q23, [x13], #32
	ldp		q26, q27, [x11]
	add		x11, x11, x12

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
	fmla	v14.2d, v20.2d, v25.d[1]
	fmla	v15.2d, v21.2d, v25.d[1]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	sub		w8, w8, #4

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q16, q17, [x9], #32
	ldp		q20, q21, [x13], #32
	ldp		q24, q25, [x11]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	cmp		w8, #0
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
	fmla	v14.2d, v20.2d, v25.d[1]
	fmla	v15.2d, v21.2d, v25.d[1]

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x9, x10

	add		x14, x12, x12
	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x13, #64]

	// preload
	ldp		q24, q25, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q26, q27, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q28, q29, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q30, q31, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #(0*8+0*32)]
	ldp		q20, q21, [x13, #(0*8+0*32)]

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #(0*8+1*32)]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q22, q23, [x13, #(0*8+1*32)]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, #128]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	prfm	PLDL1KEEP, [x13, #128]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, #192]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x13, #192]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
	prfm	PLDL1KEEP, [x11]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	prfm	PLDL1KEEP, [x11, x12]
	fmla	v14.2d, v20.2d, v25.d[1]
	fmla	v15.2d, v21.2d, v25.d[1]

	// unroll 1
	ldp		q16, q17, [x9, #(0*8+2*32)]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldp		q20, q21, [x13, #(0*8+2*32)]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	prfm	PLDL1KEEP, [x11, x14]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x11, x15]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	sub		w8, w8, #4
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	// unroll 2
	ldp		q18, q19, [x9, #(0*8+3*32)]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldp		q22, q23, [x13, #(0*8+3*32)]
	fmla	v8.2d, v20.2d, v28.d[0]
	fmla	v9.2d, v21.2d, v28.d[0]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v11.2d, v21.2d, v28.d[1]
	add		x9, x9, #128
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	add		x13, x13, #128
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v13.2d, v21.2d, v29.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v29.d[1]
	fmla	v7.2d, v17.2d, v29.d[1]
	ldp		q16, q17, [x9, #(0*8+0*32)]
	fmla	v14.2d, v20.2d, v29.d[1]
	fmla	v15.2d, v21.2d, v29.d[1]
	ldp		q20, q21, [x13, #(0*8+0*32)]

	// unroll 3
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
	fmla	v8.2d, v22.2d, v30.d[0]
	fmla	v9.2d, v23.2d, v30.d[0]
	ldp		q24, q25, [x11, #(0*8)]
	fmla	v2.2d, v18.2d, v30.d[1]
	fmla	v3.2d, v19.2d, v30.d[1]
	add		x11, x11, x12
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
	ldp		q26, q27, [x11, #(0*8)]
	fmla	v4.2d, v18.2d, v31.d[0]
	fmla	v5.2d, v19.2d, v31.d[0]
	add		x11, x11, x12
	fmla	v12.2d, v22.2d, v31.d[0]
	fmla	v13.2d, v23.2d, v31.d[0]
	ldp		q28, q29, [x11, #(0*8)]
	fmla	v6.2d, v18.2d, v31.d[1]
	fmla	v7.2d, v19.2d, v31.d[1]
	add		x11, x11, x12
	fmla	v14.2d, v22.2d, v31.d[1]
	fmla	v15.2d, v23.2d, v31.d[1]
	ldp		q30, q31, [x11, #(0*8)]
	add		x11, x11, x12

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q18, q19, [x9, #(0*8+1*32)]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	ldp		q22, q23, [x13, #(0*8+1*32)]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x13, #128]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x9, #192]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x13, #192]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x11, #128]
	fmla	v14.2d, v20.2d, v25.d[1]
	fmla	v15.2d, v21.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x11, #192]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldp		q16, q17, [x9, #(0*8+2*32)]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	ldp		q20, q21, [x13, #(0*8+2*32)]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]
//	add		x11, x11, #128
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	sub		w8, w8, #4
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	// unroll 2
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldp		q18, q19, [x9, #(0*8+3*32)]
	fmla	v8.2d, v20.2d, v28.d[0]
	fmla	v9.2d, v21.2d, v28.d[0]
	ldp		q22, q23, [x13, #(0*8+3*32)]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v11.2d, v21.2d, v28.d[1]
	add		x9, x9, #128
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	add		x13, x13, #128
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v13.2d, v21.2d, v29.d[0]
	fmla	v6.2d, v16.2d, v29.d[1]
	fmla	v7.2d, v17.2d, v29.d[1]
//	ldp		q16, q17, [x9, #(0*8+0*32)]
	fmla	v14.2d, v20.2d, v29.d[1]
	fmla	v15.2d, v21.2d, v29.d[1]
//	ldp		q20, q21, [x13, #(0*8+0*32)]

	// unroll 3
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
	fmla	v8.2d, v22.2d, v30.d[0]
	fmla	v9.2d, v23.2d, v30.d[0]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v3.2d, v19.2d, v30.d[1]
	fmla	v2.2d, v18.2d, v30.d[1]
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v31.d[0]
	fmla	v5.2d, v19.2d, v31.d[0]
	fmla	v12.2d, v22.2d, v31.d[0]
	fmla	v13.2d, v23.2d, v31.d[0]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
	fmla	v6.2d, v18.2d, v31.d[1]
	fmla	v7.2d, v19.2d, v31.d[1]
	fmla	v14.2d, v22.2d, v31.d[1]
	fmla	v15.2d, v23.2d, v31.d[1]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x9, x9, #32
//	sub		x11, x11, #32
//	sub		x13, x13, #32
	sub		x11, x11, x12
	sub		x11, x11, x12
	sub		x11, x11, x12
	sub		x11, x11, x12

3: // clean1-up loop

	// unroll 0
	ld1		{v20.2d, v21.2d}, [x9], #32
	ld1		{v28.2d, v29.2d}, [x11]
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	fmla	v6.2d, v20.2d, v29.d[1]
	fmla	v7.2d, v21.2d, v29.d[1]
	ld1		{v22.2d, v23.2d}, [x13], #32
	fmla	v8.2d, v22.2d, v28.d[0]
	fmla	v9.2d, v23.2d, v28.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v22.2d, v28.d[1]
	fmla	v11.2d, v23.2d, v28.d[1]
	cmp		w8, #0
	fmla	v12.2d, v22.2d, v29.d[0]
	fmla	v13.2d, v23.2d, v29.d[0]
	fmla	v14.2d, v22.2d, v29.d[1]
	fmla	v15.2d, v23.2d, v29.d[1]

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
// x11  <- B
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

	add		x13, x9, x10

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// main loop
1:
	
	// load 0 & 1
	ldp		q16, q17, [x9], #32
	ldp		q20, q21, [x13], #32
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	add		x11, x11, x12
	ldp		q18, q19, [x9], #32
	ldp		q22, q23, [x13], #32
	ldr		q26, [x11, #0]
	ldr		d27, [x11, #16]
	add		x11, x11, x12

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]

	// load 2 & 3
	ldp		q16, q17, [x9], #32
	ldp		q20, q21, [x13], #32
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	add		x11, x11, x12
	ldp		q18, q19, [x9], #32
	ldp		q22, q23, [x13], #32
	ldr		q26, [x11, #0]
	ldr		d27, [x11, #16]
	add		x11, x11, x12

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldp		q16, q17, [x9], #32
	ldp		q20, q21, [x13], #32
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	add		x11, x11, x12
	ldp		q18, q19, [x9], #32
	ldp		q22, q23, [x13], #32
	ldr		q26, [x11, #0]
	ldr		d27, [x11, #16]
	add		x11, x11, x12

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]

	// load 2 & 3
	ldp		q16, q17, [x9], #32
	ldp		q20, q21, [x13], #32
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	add		x11, x11, x12
	ldp		q18, q19, [x9], #32
	ldp		q22, q23, [x13], #32
	ldr		q26, [x11, #0]
	ldr		d27, [x11, #16]
	add		x11, x11, x12

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]

	sub		w8, w8, #4

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q16, q17, [x9], #32
	ldp		q20, q21, [x13], #32
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	cmp		w8, #0
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x9, x10

	add		x14, x12, x12
	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x13, #64]

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
	ldp		q16, q17, [x9, #(0*8+0*32)]
	ldp		q20, q21, [x13, #(0*8+0*32)]

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #(0*8+1*32)]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q22, q23, [x13, #(0*8+1*32)]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, #128]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	prfm	PLDL1KEEP, [x13, #128]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, #192]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x13, #192]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]

	// unroll 1
	ldp		q16, q17, [x9, #(0*8+2*32)]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldp		q20, q21, [x13, #(0*8+2*32)]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	prfm	PLDL1KEEP, [x11, x14]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x11, x15]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #(0*8+3*32)]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldp		q22, q23, [x13, #(0*8+3*32)]
	fmla	v8.2d, v20.2d, v28.d[0]
	fmla	v9.2d, v21.2d, v28.d[0]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v11.2d, v21.2d, v28.d[1]
	add		x9, x9, #128
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	add		x13, x13, #128
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v13.2d, v21.2d, v29.d[0]
	cmp		w8, #4
	ldp		q16, q17, [x9, #(0*8+0*32)]
	ldp		q20, q21, [x13, #(0*8+0*32)]

	// unroll 3
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
	fmla	v8.2d, v22.2d, v30.d[0]
	fmla	v9.2d, v23.2d, v30.d[0]
	ldr		q24, [x11, #(0*8)]
	ldr		d25, [x11, #(2*8)]
	fmla	v2.2d, v18.2d, v30.d[1]
	fmla	v3.2d, v19.2d, v30.d[1]
	add		x11, x11, x12
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
	ldr		q26, [x11, #(0*8)]
	ldr		d27, [x11, #(2*8)]
	fmla	v4.2d, v18.2d, v31.d[0]
	fmla	v5.2d, v19.2d, v31.d[0]
	add		x11, x11, x12
	fmla	v12.2d, v22.2d, v31.d[0]
	fmla	v13.2d, v23.2d, v31.d[0]
	ldr		q28, [x11, #(0*8)]
	ldr		d29, [x11, #(2*8)]
	add		x11, x11, x12
	ldr		q30, [x11, #(0*8)]
	ldr		d31, [x11, #(2*8)]
	add		x11, x11, x12

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q18, q19, [x9, #(0*8+1*32)]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	ldp		q22, q23, [x13, #(0*8+1*32)]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x13, #128]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x9, #192]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x13, #192]
//	prfm	PLDL1KEEP, [x11, #128]
//	prfm	PLDL1KEEP, [x11, #192]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldp		q16, q17, [x9, #(0*8+2*32)]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	ldp		q20, q21, [x13, #(0*8+2*32)]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]
//	add		x11, x11, #128
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldp		q18, q19, [x9, #(0*8+3*32)]
	fmla	v8.2d, v20.2d, v28.d[0]
	fmla	v9.2d, v21.2d, v28.d[0]
	ldp		q22, q23, [x13, #(0*8+3*32)]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v11.2d, v21.2d, v28.d[1]
	add		x9, x9, #128
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	add		x13, x13, #128
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v13.2d, v21.2d, v29.d[0]
//	ldp		q16, q17, [x9, #(0*8+0*32)]
//	ldp		q20, q21, [x13, #(0*8+0*32)]

	// unroll 3
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
	fmla	v8.2d, v22.2d, v30.d[0]
	fmla	v9.2d, v23.2d, v30.d[0]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v3.2d, v19.2d, v30.d[1]
	fmla	v2.2d, v18.2d, v30.d[1]
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v31.d[0]
	fmla	v5.2d, v19.2d, v31.d[0]
	fmla	v12.2d, v22.2d, v31.d[0]
	fmla	v13.2d, v23.2d, v31.d[0]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, x12
	sub		x11, x11, x12
	sub		x11, x11, x12
	sub		x11, x11, x12

3: // clean1-up loop

	// unroll 0
	ld1		{v20.2d, v21.2d}, [x9], #32
	ldr		q28, [x11, #(0*8)]
	ldr		d29, [x11, #(2*8)]
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	ld1		{v22.2d, v23.2d}, [x13], #32
	fmla	v8.2d, v22.2d, v28.d[0]
	fmla	v9.2d, v23.2d, v28.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v22.2d, v28.d[1]
	fmla	v11.2d, v23.2d, v28.d[1]
	cmp		w8, #0
	fmla	v12.2d, v22.2d, v29.d[0]
	fmla	v13.2d, v23.2d, v29.d[0]

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
// x11  <- B
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

	add		x13, x9, x10

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// main loop
1:
	
	// load 0 & 1
	ldp		q16, q17, [x9], #32
	ldp		q20, q21, [x13], #32
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldp		q18, q19, [x9], #32
	ldp		q22, q23, [x13], #32
	ldr		q26, [x11, #0]
	add		x11, x11, x12

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]

	// load 2 & 3
	ldp		q16, q17, [x9], #32
	ldp		q20, q21, [x13], #32
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldp		q18, q19, [x9], #32
	ldp		q22, q23, [x13], #32
	ldr		q26, [x11, #0]
	add		x11, x11, x12

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldp		q16, q17, [x9], #32
	ldp		q20, q21, [x13], #32
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldp		q18, q19, [x9], #32
	ldp		q22, q23, [x13], #32
	ldr		q26, [x11, #0]
	add		x11, x11, x12

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]

	// load 2 & 3
	ldp		q16, q17, [x9], #32
	ldp		q20, q21, [x13], #32
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldp		q18, q19, [x9], #32
	ldp		q22, q23, [x13], #32
	ldr		q26, [x11, #0]
	add		x11, x11, x12

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]

	sub		w8, w8, #4

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q16, q17, [x9], #32
	ldp		q20, q21, [x13], #32
	ldr		q24, [x11, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x9, x10

	add		x14, x12, x12
	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x13, #64]

	// preload
	ldr		q24, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		q26, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		q28, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		q30, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #(0*8+0*32)]
	ldp		q20, q21, [x13, #(0*8+0*32)]

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #(0*8+1*32)]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q22, q23, [x13, #(0*8+1*32)]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, #128]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	prfm	PLDL1KEEP, [x13, #128]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x13, #192]
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]

	// unroll 1
	ldp		q16, q17, [x9, #(0*8+2*32)]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldp		q20, q21, [x13, #(0*8+2*32)]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	prfm	PLDL1KEEP, [x11, x14]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x11, x15]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #(0*8+3*32)]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldp		q22, q23, [x13, #(0*8+3*32)]
	fmla	v8.2d, v20.2d, v28.d[0]
	fmla	v9.2d, v21.2d, v28.d[0]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v11.2d, v21.2d, v28.d[1]
	add		x9, x9, #128
	add		x13, x13, #128
	cmp		w8, #4
	ldp		q16, q17, [x9, #(0*8+0*32)]
	ldp		q20, q21, [x13, #(0*8+0*32)]

	// unroll 3
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
	fmla	v8.2d, v22.2d, v30.d[0]
	fmla	v9.2d, v23.2d, v30.d[0]
	ldr		q24, [x11, #(0*8)]
	fmla	v2.2d, v18.2d, v30.d[1]
	fmla	v3.2d, v19.2d, v30.d[1]
	add		x11, x11, x12
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
	ldr		q26, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		q28, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		q30, [x11, #(0*8)]
	add		x11, x11, x12

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q18, q19, [x9, #(0*8+1*32)]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	ldp		q22, q23, [x13, #(0*8+1*32)]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x13, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x13, #192]
//	prfm	PLDL1KEEP, [x11, #128]
//	prfm	PLDL1KEEP, [x11, #192]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldp		q16, q17, [x9, #(0*8+2*32)]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	ldp		q20, q21, [x13, #(0*8+2*32)]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
//	add		x11, x11, #128
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldp		q18, q19, [x9, #(0*8+3*32)]
	fmla	v8.2d, v20.2d, v28.d[0]
	fmla	v9.2d, v21.2d, v28.d[0]
	ldp		q22, q23, [x13, #(0*8+3*32)]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v11.2d, v21.2d, v28.d[1]
	add		x9, x9, #128
	add		x13, x13, #128
//	ldp		q16, q17, [x9, #(0*8+0*32)]
//	ldp		q20, q21, [x13, #(0*8+0*32)]

	// unroll 3
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
	fmla	v8.2d, v22.2d, v30.d[0]
	fmla	v9.2d, v23.2d, v30.d[0]
	fmla	v3.2d, v19.2d, v30.d[1]
	fmla	v2.2d, v18.2d, v30.d[1]
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, x12
	sub		x11, x11, x12
	sub		x11, x11, x12
	sub		x11, x11, x12

3: // clean1-up loop

	// unroll 0
	ld1		{v20.2d, v21.2d}, [x9], #32
	ld1		{v28.2d}, [x11]
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	ld1		{v22.2d, v23.2d}, [x13], #32
	fmla	v8.2d, v22.2d, v28.d[0]
	fmla	v9.2d, v23.2d, v28.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v22.2d, v28.d[1]
	fmla	v11.2d, v23.2d, v28.d[1]
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
// x11  <- B
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

	add		x13, x9, x10

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// main loop
1:
	
	// load 0 & 1
	ldp		q16, q17, [x9], #32
	ldp		q20, q21, [x13], #32
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldp		q18, q19, [x9], #32
	ldp		q22, q23, [x13], #32
	ldr		d26, [x11, #0]
	add		x11, x11, x12

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]

	// load 2 & 3
	ldp		q16, q17, [x9], #32
	ldp		q20, q21, [x13], #32
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldp		q18, q19, [x9], #32
	ldp		q22, q23, [x13], #32
	ldr		d26, [x11, #0]
	add		x11, x11, x12

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldp		q16, q17, [x9], #32
	ldp		q20, q21, [x13], #32
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldp		q18, q19, [x9], #32
	ldp		q22, q23, [x13], #32
	ldr		d26, [x11, #0]
	add		x11, x11, x12

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]

	// load 2 & 3
	ldp		q16, q17, [x9], #32
	ldp		q20, q21, [x13], #32
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldp		q18, q19, [x9], #32
	ldp		q22, q23, [x13], #32
	ldr		d26, [x11, #0]
	add		x11, x11, x12

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]

	sub		w8, w8, #4

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q16, q17, [x9], #32
	ldp		q20, q21, [x13], #32
	ldr		d24, [x11, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	add		x11, x11, x12
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x9, x10

	add		x14, x12, x12
	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x13, #64]

	// preload
	ldr		d24, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		d26, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		d28, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		d30, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #(0*8+0*32)]
	ldp		q20, q21, [x13, #(0*8+0*32)]

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #(0*8+1*32)]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q22, q23, [x13, #(0*8+1*32)]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, #128]
	prfm	PLDL1KEEP, [x13, #128]
	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x13, #192]
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]

	// unroll 1
	ldp		q16, q17, [x9, #(0*8+2*32)]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldp		q20, q21, [x13, #(0*8+2*32)]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #(0*8+3*32)]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldp		q22, q23, [x13, #(0*8+3*32)]
	fmla	v8.2d, v20.2d, v28.d[0]
	fmla	v9.2d, v21.2d, v28.d[0]
	add		x9, x9, #128
	add		x13, x13, #128
	cmp		w8, #4
	ldp		q16, q17, [x9, #(0*8+0*32)]
	ldp		q20, q21, [x13, #(0*8+0*32)]

	// unroll 3
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
	fmla	v8.2d, v22.2d, v30.d[0]
	fmla	v9.2d, v23.2d, v30.d[0]
	ldr		d24, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		d26, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		d28, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		d30, [x11, #(0*8)]
	add		x11, x11, x12

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q18, q19, [x9, #(0*8+1*32)]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	ldp		q22, q23, [x13, #(0*8+1*32)]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x13, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x13, #192]
//	prfm	PLDL1KEEP, [x11, #128]
//	prfm	PLDL1KEEP, [x11, #192]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldp		q16, q17, [x9, #(0*8+2*32)]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	ldp		q20, q21, [x13, #(0*8+2*32)]
//	add		x11, x11, #128
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldp		q18, q19, [x9, #(0*8+3*32)]
	fmla	v8.2d, v20.2d, v28.d[0]
	fmla	v9.2d, v21.2d, v28.d[0]
	ldp		q22, q23, [x13, #(0*8+3*32)]
	add		x9, x9, #128
	add		x13, x13, #128
//	ldp		q16, q17, [x9, #(0*8+0*32)]
//	ldp		q20, q21, [x13, #(0*8+0*32)]

	// unroll 3
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
	fmla	v8.2d, v22.2d, v30.d[0]
	fmla	v9.2d, v23.2d, v30.d[0]
//	ldr		d24, [x11, #(0*8+0*32)]
//	ldr		d26, [x11, #(0*8+1*32)]
//	ldr		d28, [x11, #(0*8+2*32)]
//	ldr		d30, [x11, #(0*8+3*32)]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, x12
	sub		x11, x11, x12
	sub		x11, x11, x12
	sub		x11, x11, x12

3: // clean1-up loop

	// unroll 0
	ld1		{v20.2d, v21.2d}, [x9], #32
	ldr		d28, [x11]
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	add		x11, x11, x12
	ld1		{v22.2d, v23.2d}, [x13], #32
	fmla	v8.2d, v22.2d, v28.d[0]
	fmla	v9.2d, v23.2d, v28.d[0]
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
// x11   <- B
// x12   <- 8*ldb
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

	add		x13, x9, x10

	add		x14, x11, x12
	add		x15, x14, x12
	add		x16, x15, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldr		q27, [x16], #16
	ldp		q16, q17, [x9], #32
	ldp		q18, q19, [x9], #32
	ldp		q20, q21, [x13], #32
	ldp		q22, q23, [x13], #32

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v13.2d, v21.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v14.2d, v20.2d, v27.d[0]
	fmla	v15.2d, v21.2d, v27.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v13.2d, v23.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldr		q27, [x16], #16
	ldp		q16, q17, [x9], #32
	ldp		q18, q19, [x9], #32
	ldp		q20, q21, [x13], #32
	ldp		q22, q23, [x13], #32

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v13.2d, v21.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v14.2d, v20.2d, v27.d[0]
	fmla	v15.2d, v21.2d, v27.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v13.2d, v23.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldr		q27, [x16], #16
	ldp		q16, q17, [x9], #32
	ldp		q18, q19, [x9], #32
	ldp		q20, q21, [x13], #32
	ldp		q22, q23, [x13], #32

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v13.2d, v21.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v14.2d, v20.2d, v27.d[0]
	fmla	v15.2d, v21.2d, v27.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v13.2d, v23.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldr		q27, [x16], #16
	ldp		q16, q17, [x9], #32
	ldp		q18, q19, [x9], #32
	ldp		q20, q21, [x13], #32
	ldp		q22, q23, [x13], #32

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v13.2d, v21.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v14.2d, v20.2d, v27.d[0]
	fmla	v15.2d, v21.2d, v27.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v13.2d, v23.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x13, #0]
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8
	ldr		d30, [x15], #8
	ldr		d31, [x16], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v9.2d, v27.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	add		x9, x9, #32
	fmla	v10.2d, v26.2d, v29.d[0]
	fmla	v11.2d, v27.2d, v29.d[0]
	add		x13, x13, #32
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
//	add		x11, x11, #8
	fmla	v12.2d, v26.2d, v30.d[0]
	fmla	v13.2d, v27.2d, v30.d[0]
	sub		w8, w8, #1
	fmla	v6.2d, v24.2d, v31.d[0]
	fmla	v7.2d, v25.2d, v31.d[0]
	cmp		w8, #0
	fmla	v14.2d, v26.2d, v31.d[0]
	fmla	v15.2d, v27.2d, v31.d[0]

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x9, x10

	add		x14, x11, x12
	add		x15, x14, x12
	add		x16, x15, x12

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x13, #64]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x14], #32
	ldp		q28, q29, [x15], #32
	ldp		q30, q31, [x16], #32
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x13, #0]

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch
//	add		x14, x12, #64

//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x14]


	// main loop
1:
	
	ldp		q18, q19, [x9, #32]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q22, q23, [x13, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, #128]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	prfm	PLDL1KEEP, [x13, #128]
	fmla	v10.2d, v20.2d, v26.d[0]
	fmla	v11.2d, v21.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, #192]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x13, #192]
	fmla	v12.2d, v20.2d, v28.d[0]
	fmla	v13.2d, v21.2d, v28.d[0]
	prfm	PLDL1KEEP, [x11, #32]
	fmla	v6.2d, v16.2d, v30.d[0]
	fmla	v7.2d, v17.2d, v30.d[0]
	prfm	PLDL1KEEP, [x14, #32]
	fmla	v14.2d, v20.2d, v30.d[0]
	fmla	v15.2d, v21.2d, v30.d[0]

	ldp		q16, q17, [x9, #64]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldp		q20, q21, [x13, #64]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	prfm	PLDL1KEEP, [x15, #32]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x16, #32]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	fmla	v12.2d, v22.2d, v28.d[1]
	fmla	v13.2d, v23.2d, v28.d[1]
//	add		x11, x11, x12
	fmla	v6.2d, v18.2d, v30.d[1]
	fmla	v7.2d, v19.2d, v30.d[1]
	sub		w8, w8, #4
	fmla	v14.2d, v22.2d, v30.d[1]
	fmla	v15.2d, v23.2d, v30.d[1]

	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	ldp		q18, q19, [x9, #96]
	fmla	v8.2d, v20.2d, v25.d[0]
	fmla	v9.2d, v21.2d, v25.d[0]
	ldp		q22, q23, [x13, #96]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	fmla	v10.2d, v20.2d, v27.d[0]
	fmla	v11.2d, v21.2d, v27.d[0]
	add		x9, x9, #128
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	add		x13, x13, #128
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v13.2d, v21.2d, v29.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v31.d[0]
	fmla	v7.2d, v17.2d, v31.d[0]
	ldp		q16, q17, [x9, #0]
	fmla	v14.2d, v20.2d, v31.d[0]
	fmla	v15.2d, v21.2d, v31.d[0]
	ldp		q20, q21, [x13, #0]

	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	fmla	v8.2d, v22.2d, v25.d[1]
	fmla	v9.2d, v23.2d, v25.d[1]
	ldp		q24, q25, [x11], #32
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	fmla	v10.2d, v22.2d, v27.d[1]
	fmla	v11.2d, v23.2d, v27.d[1]
	ldp		q26, q27, [x14], #32
	fmla	v4.2d, v18.2d, v29.d[1]
	fmla	v5.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v22.2d, v29.d[1]
	fmla	v13.2d, v23.2d, v29.d[1]
	ldp		q28, q29, [x15], #32
	fmla	v6.2d, v18.2d, v31.d[1]
	fmla	v7.2d, v19.2d, v31.d[1]
	fmla	v14.2d, v22.2d, v31.d[1]
	fmla	v15.2d, v23.2d, v31.d[1]
	ldp		q30, q31, [x16], #32

	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q18, q19, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	ldp		q22, q23, [x13, #32]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x12]
	fmla	v10.2d, v20.2d, v26.d[0]
	fmla	v11.2d, v21.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x14]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v12.2d, v20.2d, v28.d[0]
	fmla	v13.2d, v21.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x13, #128]
	fmla	v6.2d, v16.2d, v30.d[0]
	fmla	v7.2d, v17.2d, v30.d[0]
//	prfm	PLDL1KEEP, [x9, #192]
	fmla	v14.2d, v20.2d, v30.d[0]
	fmla	v15.2d, v21.2d, v30.d[0]
//	prfm	PLDL1KEEP, [x13, #192]

	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldp		q16, q17, [x9, #64]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	ldp		q20, q21, [x13, #64]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	fmla	v12.2d, v22.2d, v28.d[1]
	fmla	v13.2d, v23.2d, v28.d[1]
//	add		x11, x11, x12
	fmla	v6.2d, v18.2d, v30.d[1]
	fmla	v7.2d, v19.2d, v30.d[1]
	sub		w8, w8, #4
	fmla	v14.2d, v22.2d, v30.d[1]
	fmla	v15.2d, v23.2d, v30.d[1]

	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	ldp		q18, q19, [x9, #96]
	fmla	v8.2d, v20.2d, v25.d[0]
	fmla	v9.2d, v21.2d, v25.d[0]
	ldp		q22, q23, [x13, #96]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	fmla	v10.2d, v20.2d, v27.d[0]
	fmla	v11.2d, v21.2d, v27.d[0]
	add		x9, x9, #128
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	add		x13, x13, #128
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v13.2d, v21.2d, v29.d[0]
	fmla	v6.2d, v16.2d, v31.d[0]
	fmla	v7.2d, v17.2d, v31.d[0]
//	ldp		q16, q17, [x9, #0]
	fmla	v14.2d, v20.2d, v31.d[0]
	fmla	v15.2d, v21.2d, v31.d[0]
//	ldp		q20, q21, [x13, #0]

	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	fmla	v8.2d, v22.2d, v25.d[1]
	fmla	v9.2d, v23.2d, v25.d[1]
//	ldp		q24, q25, [x11, #0]
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	fmla	v10.2d, v22.2d, v27.d[1]
	fmla	v11.2d, v23.2d, v27.d[1]
//	ldp		q26, q27, [x11, #32]
	fmla	v4.2d, v18.2d, v29.d[1]
	fmla	v5.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v22.2d, v29.d[1]
	fmla	v13.2d, v23.2d, v29.d[1]
//	ldp		q28, q29, [x11, #64]
	fmla	v6.2d, v18.2d, v31.d[1]
	fmla	v7.2d, v19.2d, v31.d[1]
	fmla	v14.2d, v22.2d, v31.d[1]
	fmla	v15.2d, v23.2d, v31.d[1]
//	ldp		q30, q31, [x11, #96]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x16, x16, #32

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x13, #0]
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8
	ldr		d30, [x15], #8
	ldr		d31, [x16], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v9.2d, v27.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	add		x9, x9, #32
	fmla	v10.2d, v26.2d, v29.d[0]
	fmla	v11.2d, v27.2d, v29.d[0]
	add		x13, x13, #32
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
//	add		x11, x11, #8
	fmla	v12.2d, v26.2d, v30.d[0]
	fmla	v13.2d, v27.2d, v30.d[0]
	sub		w8, w8, #1
	fmla	v6.2d, v24.2d, v31.d[0]
	fmla	v7.2d, v25.2d, v31.d[0]
	cmp		w8, #0
	fmla	v14.2d, v26.2d, v31.d[0]
	fmla	v15.2d, v27.2d, v31.d[0]

	bgt		3b

2: // return



#endif
	


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
// x11   <- B
// x12   <- 8*ldb
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

	add		x13, x9, x10

	add		x14, x11, x12
	add		x15, x14, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldp		q16, q17, [x9], #32
	ldp		q18, q19, [x9], #32
	ldp		q20, q21, [x13], #32
	ldp		q22, q23, [x13], #32

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v13.2d, v21.2d, v26.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v13.2d, v23.2d, v26.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldp		q16, q17, [x9], #32
	ldp		q18, q19, [x9], #32
	ldp		q20, q21, [x13], #32
	ldp		q22, q23, [x13], #32

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v13.2d, v21.2d, v26.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v13.2d, v23.2d, v26.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldp		q16, q17, [x9], #32
	ldp		q18, q19, [x9], #32
	ldp		q20, q21, [x13], #32
	ldp		q22, q23, [x13], #32

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v13.2d, v21.2d, v26.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v13.2d, v23.2d, v26.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldp		q16, q17, [x9], #32
	ldp		q18, q19, [x9], #32
	ldp		q20, q21, [x13], #32
	ldp		q22, q23, [x13], #32

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v13.2d, v21.2d, v26.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v13.2d, v23.2d, v26.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x13, #0]
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8
	ldr		d30, [x15], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v9.2d, v27.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	add		x9, x9, #32
	fmla	v10.2d, v26.2d, v29.d[0]
	fmla	v11.2d, v27.2d, v29.d[0]
	add		x13, x13, #32
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
//	add		x11, x11, #8
	fmla	v12.2d, v26.2d, v30.d[0]
	fmla	v13.2d, v27.2d, v30.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x9, x10

	add		x14, x11, x12
	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x13, #64]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x14], #32
	ldp		q28, q29, [x15], #32
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x13, #0]

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch

	// main loop
1:
	
	ldp		q18, q19, [x9, #32]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q22, q23, [x13, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, #128]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	prfm	PLDL1KEEP, [x13, #128]
	fmla	v10.2d, v20.2d, v26.d[0]
	fmla	v11.2d, v21.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, #192]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x13, #192]
	fmla	v12.2d, v20.2d, v28.d[0]
	fmla	v13.2d, v21.2d, v28.d[0]
	prfm	PLDL1KEEP, [x11, #32]
	prfm	PLDL1KEEP, [x14, #32]

	ldp		q16, q17, [x9, #64]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldp		q20, q21, [x13, #64]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	prfm	PLDL1KEEP, [x15, #32]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
//	prfm	PLDL1KEEP, [x16, #32]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	fmla	v12.2d, v22.2d, v28.d[1]
	fmla	v13.2d, v23.2d, v28.d[1]
//	add		x11, x11, x12
	sub		w8, w8, #4

	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	ldp		q18, q19, [x9, #96]
	fmla	v8.2d, v20.2d, v25.d[0]
	fmla	v9.2d, v21.2d, v25.d[0]
	ldp		q22, q23, [x13, #96]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	fmla	v10.2d, v20.2d, v27.d[0]
	fmla	v11.2d, v21.2d, v27.d[0]
	add		x9, x9, #128
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	add		x13, x13, #128
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v13.2d, v21.2d, v29.d[0]
	cmp		w8, #4
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x13, #0]

	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	fmla	v8.2d, v22.2d, v25.d[1]
	fmla	v9.2d, v23.2d, v25.d[1]
	ldp		q24, q25, [x11], #32
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	fmla	v10.2d, v22.2d, v27.d[1]
	fmla	v11.2d, v23.2d, v27.d[1]
	ldp		q26, q27, [x14], #32
	fmla	v4.2d, v18.2d, v29.d[1]
	fmla	v5.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v22.2d, v29.d[1]
	fmla	v13.2d, v23.2d, v29.d[1]
	ldp		q28, q29, [x15], #32

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q18, q19, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	ldp		q22, q23, [x13, #32]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x12]
	fmla	v10.2d, v20.2d, v26.d[0]
	fmla	v11.2d, v21.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x14]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v12.2d, v20.2d, v28.d[0]
	fmla	v13.2d, v21.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x13, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x13, #192]

	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldp		q16, q17, [x9, #64]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	ldp		q20, q21, [x13, #64]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	fmla	v12.2d, v22.2d, v28.d[1]
	fmla	v13.2d, v23.2d, v28.d[1]
//	add		x11, x11, x12
	sub		w8, w8, #4

	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	ldp		q18, q19, [x9, #96]
	fmla	v8.2d, v20.2d, v25.d[0]
	fmla	v9.2d, v21.2d, v25.d[0]
	ldp		q22, q23, [x13, #96]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	fmla	v10.2d, v20.2d, v27.d[0]
	fmla	v11.2d, v21.2d, v27.d[0]
	add		x9, x9, #128
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	add		x13, x13, #128
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v13.2d, v21.2d, v29.d[0]
//	ldp		q16, q17, [x9, #0]
//	ldp		q20, q21, [x13, #0]

	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	fmla	v8.2d, v22.2d, v25.d[1]
	fmla	v9.2d, v23.2d, v25.d[1]
//	ldp		q24, q25, [x11, #0]
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	fmla	v10.2d, v22.2d, v27.d[1]
	fmla	v11.2d, v23.2d, v27.d[1]
//	ldp		q26, q27, [x11, #32]
	fmla	v4.2d, v18.2d, v29.d[1]
	fmla	v5.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v22.2d, v29.d[1]
	fmla	v13.2d, v23.2d, v29.d[1]
//	ldp		q28, q29, [x11, #64]
//	ldp		q30, q31, [x11, #96]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x14, x14, #32
	sub		x15, x15, #32

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x13, #0]
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8
	ldr		d30, [x15], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v9.2d, v27.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	add		x9, x9, #32
	fmla	v10.2d, v26.2d, v29.d[0]
	fmla	v11.2d, v27.2d, v29.d[0]
	add		x13, x13, #32
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
//	add		x11, x11, #8
	fmla	v12.2d, v26.2d, v30.d[0]
	fmla	v13.2d, v27.2d, v30.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


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
// x11   <- B
// x12   <- 8*ldb
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

	add		x13, x9, x10

	add		x14, x11, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldp		q16, q17, [x9], #32
	ldp		q18, q19, [x9], #32
	ldp		q20, q21, [x13], #32
	ldp		q22, q23, [x13], #32

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldp		q16, q17, [x9], #32
	ldp		q18, q19, [x9], #32
	ldp		q20, q21, [x13], #32
	ldp		q22, q23, [x13], #32

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldp		q16, q17, [x9], #32
	ldp		q18, q19, [x9], #32
	ldp		q20, q21, [x13], #32
	ldp		q22, q23, [x13], #32

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldp		q16, q17, [x9], #32
	ldp		q18, q19, [x9], #32
	ldp		q20, q21, [x13], #32
	ldp		q22, q23, [x13], #32

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x13, #0]
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v9.2d, v27.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	add		x9, x9, #32
	fmla	v10.2d, v26.2d, v29.d[0]
	fmla	v11.2d, v27.2d, v29.d[0]
	add		x13, x13, #32
//	add		x11, x11, #8
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x9, x10

	add		x14, x11, x12

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x13, #64]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x14], #32
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x13, #0]

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch

	// main loop
1:
	
	ldp		q18, q19, [x9, #32]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q22, q23, [x13, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, #128]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	prfm	PLDL1KEEP, [x13, #128]
	fmla	v10.2d, v20.2d, v26.d[0]
	fmla	v11.2d, v21.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x13, #192]
	prfm	PLDL1KEEP, [x11, #32]
	prfm	PLDL1KEEP, [x14, #32]

	ldp		q16, q17, [x9, #64]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldp		q20, q21, [x13, #64]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x15, #32]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
//	prfm	PLDL1KEEP, [x16, #32]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
//	add		x11, x11, x12
	sub		w8, w8, #4

	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	ldp		q18, q19, [x9, #96]
	fmla	v8.2d, v20.2d, v25.d[0]
	fmla	v9.2d, v21.2d, v25.d[0]
	ldp		q22, q23, [x13, #96]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	fmla	v10.2d, v20.2d, v27.d[0]
	fmla	v11.2d, v21.2d, v27.d[0]
	add		x9, x9, #128
	add		x13, x13, #128
	cmp		w8, #4
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x13, #0]

	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	fmla	v8.2d, v22.2d, v25.d[1]
	fmla	v9.2d, v23.2d, v25.d[1]
	ldp		q24, q25, [x11], #32
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	fmla	v10.2d, v22.2d, v27.d[1]
	fmla	v11.2d, v23.2d, v27.d[1]
	ldp		q26, q27, [x14], #32

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q18, q19, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	ldp		q22, q23, [x13, #32]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x12]
	fmla	v10.2d, v20.2d, v26.d[0]
	fmla	v11.2d, v21.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x13, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x13, #192]

	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldp		q16, q17, [x9, #64]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	ldp		q20, q21, [x13, #64]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
//	add		x11, x11, x12
	sub		w8, w8, #4

	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	ldp		q18, q19, [x9, #96]
	fmla	v8.2d, v20.2d, v25.d[0]
	fmla	v9.2d, v21.2d, v25.d[0]
	ldp		q22, q23, [x13, #96]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	fmla	v10.2d, v20.2d, v27.d[0]
	fmla	v11.2d, v21.2d, v27.d[0]
	add		x9, x9, #128
	add		x13, x13, #128

	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	fmla	v8.2d, v22.2d, v25.d[1]
	fmla	v9.2d, v23.2d, v25.d[1]
//	ldp		q24, q25, [x11, #0]
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	fmla	v10.2d, v22.2d, v27.d[1]
	fmla	v11.2d, v23.2d, v27.d[1]
//	ldp		q26, q27, [x11, #32]
//	ldp		q28, q29, [x11, #64]
//	ldp		q30, q31, [x11, #96]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x14, x14, #32

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x13, #0]
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v9.2d, v27.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	add		x9, x9, #32
	fmla	v10.2d, v26.2d, v29.d[0]
	fmla	v11.2d, v27.2d, v29.d[0]
	add		x13, x13, #32
//	add		x11, x11, #8
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


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
// x11   <- B
// x12   <- 8*ldb
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

	add		x13, x9, x10

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldp		q16, q17, [x9], #32
	ldp		q18, q19, [x9], #32
	ldp		q20, q21, [x13], #32
	ldp		q22, q23, [x13], #32

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldp		q16, q17, [x9], #32
	ldp		q18, q19, [x9], #32
	ldp		q20, q21, [x13], #32
	ldp		q22, q23, [x13], #32

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldp		q16, q17, [x9], #32
	ldp		q18, q19, [x9], #32
	ldp		q20, q21, [x13], #32
	ldp		q22, q23, [x13], #32

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldp		q16, q17, [x9], #32
	ldp		q18, q19, [x9], #32
	ldp		q20, q21, [x13], #32
	ldp		q22, q23, [x13], #32

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x13, #0]
	ldr		d28, [x11], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v9.2d, v27.2d, v28.d[0]
	add		x9, x9, #32
	add		x13, x13, #32
//	add		x11, x11, #8
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x9, x10

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x13, #64]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x13, #0]

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// main loop
1:
	
	ldp		q18, q19, [x9, #32]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q22, q23, [x13, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, #128]
	prfm	PLDL1KEEP, [x13, #128]
	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x13, #192]
	prfm	PLDL1KEEP, [x11, #32]
//	prfm	PLDL1KEEP, [x14, #32]

	ldp		q16, q17, [x9, #64]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldp		q20, q21, [x13, #64]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x15, #32]
//	prfm	PLDL1KEEP, [x16, #32]
//	add		x11, x11, x12
	sub		w8, w8, #4

	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	ldp		q18, q19, [x9, #96]
	fmla	v8.2d, v20.2d, v25.d[0]
	fmla	v9.2d, v21.2d, v25.d[0]
	ldp		q22, q23, [x13, #96]
	add		x9, x9, #128
	add		x13, x13, #128
	cmp		w8, #4
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x13, #0]

	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	fmla	v8.2d, v22.2d, v25.d[1]
	fmla	v9.2d, v23.2d, v25.d[1]
	ldp		q24, q25, [x11], #32

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q18, q19, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	ldp		q22, q23, [x13, #32]
//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x13, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x13, #192]

	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldp		q16, q17, [x9, #64]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	ldp		q20, q21, [x13, #64]
//	add		x11, x11, x12
	sub		w8, w8, #4

	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	ldp		q18, q19, [x9, #96]
	fmla	v8.2d, v20.2d, v25.d[0]
	fmla	v9.2d, v21.2d, v25.d[0]
	ldp		q22, q23, [x13, #96]
	add		x9, x9, #128
	add		x13, x13, #128
//	ldp		q16, q17, [x9, #0]
//	ldp		q20, q21, [x13, #0]

	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	fmla	v8.2d, v22.2d, v25.d[1]
	fmla	v9.2d, v23.2d, v25.d[1]
//	ldp		q24, q25, [x11, #0]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x13, #0]
	ldr		d28, [x11], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v9.2d, v27.2d, v28.d[0]
	add		x9, x9, #32
	add		x13, x13, #32
//	add		x11, x11, #8
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_8x1_lib4c)
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
	.macro INNER_KERNEL_GEMM_ADD_NT_8X4_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_8x4_libcc)
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

	// main loop
1:
	
	// load 0 & 1
	ldp		q24, q25, [x11]
	add		x11, x11, x12
	ldp		q26, q27, [x11]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
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
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
	fmla	v14.2d, v20.2d, v25.d[1]
	fmla	v15.2d, v21.2d, v25.d[1]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	// load 2 & 3
	ldp		q24, q25, [x11]
	add		x11, x11, x12
	ldp		q26, q27, [x11]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
	fmla	v14.2d, v20.2d, v25.d[1]
	fmla	v15.2d, v21.2d, v25.d[1]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldp		q24, q25, [x11]
	add		x11, x11, x12
	ldp		q26, q27, [x11]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
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
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
	fmla	v14.2d, v20.2d, v25.d[1]
	fmla	v15.2d, v21.2d, v25.d[1]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	// load 2 & 3
	ldp		q24, q25, [x11]
	add		x11, x11, x12
	ldp		q26, q27, [x11]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
	fmla	v14.2d, v20.2d, v25.d[1]
	fmla	v15.2d, v21.2d, v25.d[1]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	sub		w8, w8, #4

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q24, q25, [x11]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	cmp		w8, #0
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
	fmla	v14.2d, v20.2d, v25.d[1]
	fmla	v15.2d, v21.2d, v25.d[1]

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	add		x14, x12, x12
	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
	ldp		q24, q25, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q26, q27, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q28, q29, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q30, q31, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]


	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q22, q23, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	prfm	PLDL1KEEP, [x11]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x11, x12]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
	prfm	PLDL1KEEP, [x11, x14]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	prfm	PLDL1KEEP, [x11, x15]
	fmla	v14.2d, v20.2d, v25.d[1]
	fmla	v15.2d, v21.2d, v25.d[1]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldp		q20, q21, [x9, #32]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	sub		w8, w8, #4
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldp		q22, q23, [x9, #32]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v28.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v11.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v13.2d, v21.2d, v29.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v29.d[1]
	fmla	v7.2d, v17.2d, v29.d[1]
	ldp		q16, q17, [x9, #0]
	fmla	v14.2d, v20.2d, v29.d[1]
	fmla	v15.2d, v21.2d, v29.d[1]
	ldp		q20, q21, [x9, #32]

	// unroll 3
	fmla	v0.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v8.2d, v22.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v9.2d, v23.2d, v30.d[0]
	ldp		q24, q25, [x11, #(0*8)]
	fmla	v2.2d, v18.2d, v30.d[1]
	fmla	v3.2d, v19.2d, v30.d[1]
	add		x11, x11, x12
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
	ldp		q26, q27, [x11, #(0*8)]
	fmla	v4.2d, v18.2d, v31.d[0]
	fmla	v5.2d, v19.2d, v31.d[0]
	add		x11, x11, x12
	fmla	v12.2d, v22.2d, v31.d[0]
	fmla	v13.2d, v23.2d, v31.d[0]
	ldp		q28, q29, [x11, #(0*8)]
	fmla	v6.2d, v18.2d, v31.d[1]
	fmla	v7.2d, v19.2d, v31.d[1]
	add		x11, x11, x12
	fmla	v14.2d, v22.2d, v31.d[1]
	fmla	v15.2d, v23.2d, v31.d[1]
	ldp		q30, q31, [x11, #(0*8)]
	add		x11, x11, x12

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q22, q23, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x13, #128]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x9, #192]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x13, #192]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x11, #128]
	fmla	v14.2d, v20.2d, v25.d[1]
	fmla	v15.2d, v21.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x11, #192]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldp		q20, q21, [x9, #32]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]
//	add		x11, x11, #128
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	sub		w8, w8, #4
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldp		q22, q23, [x9, #32]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v28.d[0]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v11.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v13.2d, v21.2d, v29.d[0]
	fmla	v6.2d, v16.2d, v29.d[1]
	fmla	v7.2d, v17.2d, v29.d[1]
	fmla	v14.2d, v20.2d, v29.d[1]
	fmla	v15.2d, v21.2d, v29.d[1]

	// unroll 3
//	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldp		q20, q21, [x9, #32]
	fmla	v8.2d, v22.2d, v30.d[0]
//	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v30.d[0]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v3.2d, v19.2d, v30.d[1]
	fmla	v2.2d, v18.2d, v30.d[1]
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v31.d[0]
	fmla	v5.2d, v19.2d, v31.d[0]
	fmla	v12.2d, v22.2d, v31.d[0]
	fmla	v13.2d, v23.2d, v31.d[0]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
	fmla	v6.2d, v18.2d, v31.d[1]
	fmla	v7.2d, v19.2d, v31.d[1]
	fmla	v14.2d, v22.2d, v31.d[1]
	fmla	v15.2d, v23.2d, v31.d[1]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

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
	ldp		q20, q21, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10
	ldp		q28, q29, [x11]
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	fmla	v6.2d, v20.2d, v29.d[1]
	fmla	v7.2d, v21.2d, v29.d[1]
	fmla	v8.2d, v22.2d, v28.d[0]
	fmla	v9.2d, v23.2d, v28.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v22.2d, v28.d[1]
	fmla	v11.2d, v23.2d, v28.d[1]
	cmp		w8, #0
	fmla	v12.2d, v22.2d, v29.d[0]
	fmla	v13.2d, v23.2d, v29.d[0]
	fmla	v14.2d, v22.2d, v29.d[1]
	fmla	v15.2d, v23.2d, v29.d[1]

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_8x4_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NT_7X4_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_7x4_libcc)
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

	// main loop
1:
	
	// load 0 & 1
	ldp		q24, q25, [x11]
	add		x11, x11, x12
	ldp		q26, q27, [x11]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
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
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
	fmla	v14.2d, v20.2d, v25.d[1]
	fmla	v15.2d, v21.2d, v25.d[1]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	// load 2 & 3
	ldp		q24, q25, [x11]
	add		x11, x11, x12
	ldp		q26, q27, [x11]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
	fmla	v14.2d, v20.2d, v25.d[1]
	fmla	v15.2d, v21.2d, v25.d[1]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldp		q24, q25, [x11]
	add		x11, x11, x12
	ldp		q26, q27, [x11]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
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
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
	fmla	v14.2d, v20.2d, v25.d[1]
	fmla	v15.2d, v21.2d, v25.d[1]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	// load 2 & 3
	ldp		q24, q25, [x11]
	add		x11, x11, x12
	ldp		q26, q27, [x11]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
	fmla	v14.2d, v20.2d, v25.d[1]
	fmla	v15.2d, v21.2d, v25.d[1]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	sub		w8, w8, #4

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q24, q25, [x11]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	cmp		w8, #0
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
	fmla	v14.2d, v20.2d, v25.d[1]
	fmla	v15.2d, v21.2d, v25.d[1]

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	add		x14, x12, x12
	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
	ldp		q24, q25, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q26, q27, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q28, q29, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q30, q31, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]


	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	prfm	PLDL1KEEP, [x11]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x11, x12]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
	prfm	PLDL1KEEP, [x11, x14]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	prfm	PLDL1KEEP, [x11, x15]
	fmla	v14.2d, v20.2d, v25.d[1]
	fmla	v15.2d, v21.2d, v25.d[1]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	sub		w8, w8, #4
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v28.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v11.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v13.2d, v21.2d, v29.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v29.d[1]
	fmla	v7.2d, v17.2d, v29.d[1]
	ldp		q16, q17, [x9, #0]
	fmla	v14.2d, v20.2d, v29.d[1]
	fmla	v15.2d, v21.2d, v29.d[1]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]

	// unroll 3
	fmla	v0.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v8.2d, v22.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v9.2d, v23.2d, v30.d[0]
	ldp		q24, q25, [x11, #(0*8)]
	fmla	v2.2d, v18.2d, v30.d[1]
	fmla	v3.2d, v19.2d, v30.d[1]
	add		x11, x11, x12
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
	ldp		q26, q27, [x11, #(0*8)]
	fmla	v4.2d, v18.2d, v31.d[0]
	fmla	v5.2d, v19.2d, v31.d[0]
	add		x11, x11, x12
	fmla	v12.2d, v22.2d, v31.d[0]
	fmla	v13.2d, v23.2d, v31.d[0]
	ldp		q28, q29, [x11, #(0*8)]
	fmla	v6.2d, v18.2d, v31.d[1]
	fmla	v7.2d, v19.2d, v31.d[1]
	add		x11, x11, x12
	fmla	v14.2d, v22.2d, v31.d[1]
	fmla	v15.2d, v23.2d, v31.d[1]
	ldp		q30, q31, [x11, #(0*8)]
	add		x11, x11, x12

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x13, #128]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x9, #192]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x13, #192]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x11, #128]
	fmla	v14.2d, v20.2d, v25.d[1]
	fmla	v15.2d, v21.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x11, #192]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]
//	add		x11, x11, #128
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	sub		w8, w8, #4
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v28.d[0]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v11.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v13.2d, v21.2d, v29.d[0]
	fmla	v6.2d, v16.2d, v29.d[1]
	fmla	v7.2d, v17.2d, v29.d[1]
	fmla	v14.2d, v20.2d, v29.d[1]
	fmla	v15.2d, v21.2d, v29.d[1]

	// unroll 3
//	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldr		q20, [x9, #32]
//	ldr		d21, [x9, #48]
	fmla	v8.2d, v22.2d, v30.d[0]
//	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v30.d[0]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v3.2d, v19.2d, v30.d[1]
	fmla	v2.2d, v18.2d, v30.d[1]
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v31.d[0]
	fmla	v5.2d, v19.2d, v31.d[0]
	fmla	v12.2d, v22.2d, v31.d[0]
	fmla	v13.2d, v23.2d, v31.d[0]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
	fmla	v6.2d, v18.2d, v31.d[1]
	fmla	v7.2d, v19.2d, v31.d[1]
	fmla	v14.2d, v22.2d, v31.d[1]
	fmla	v15.2d, v23.2d, v31.d[1]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

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
	ldp		q20, q21, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10
	ldp		q28, q29, [x11]
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	fmla	v6.2d, v20.2d, v29.d[1]
	fmla	v7.2d, v21.2d, v29.d[1]
	fmla	v8.2d, v22.2d, v28.d[0]
	fmla	v9.2d, v23.2d, v28.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v22.2d, v28.d[1]
	fmla	v11.2d, v23.2d, v28.d[1]
	cmp		w8, #0
	fmla	v12.2d, v22.2d, v29.d[0]
	fmla	v13.2d, v23.2d, v29.d[0]
	fmla	v14.2d, v22.2d, v29.d[1]
	fmla	v15.2d, v23.2d, v29.d[1]

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_7x4_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NT_6X4_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_6x4_libcc)
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

	// main loop
1:
	
	// load 0 & 1
	ldp		q24, q25, [x11]
	add		x11, x11, x12
	ldp		q26, q27, [x11]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
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
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v14.2d, v20.2d, v25.d[1]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v14.2d, v22.2d, v27.d[1]

	// load 2 & 3
	ldp		q24, q25, [x11]
	add		x11, x11, x12
	ldp		q26, q27, [x11]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v14.2d, v20.2d, v25.d[1]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v14.2d, v22.2d, v27.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldp		q24, q25, [x11]
	add		x11, x11, x12
	ldp		q26, q27, [x11]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
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
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v14.2d, v20.2d, v25.d[1]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v14.2d, v22.2d, v27.d[1]

	// load 2 & 3
	ldp		q24, q25, [x11]
	add		x11, x11, x12
	ldp		q26, q27, [x11]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v14.2d, v20.2d, v25.d[1]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v14.2d, v22.2d, v27.d[1]

	sub		w8, w8, #4

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q24, q25, [x11]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v20.2d, v24.d[1]
	cmp		w8, #0
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v14.2d, v20.2d, v25.d[1]

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	add		x14, x12, x12
	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
	ldp		q24, q25, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q26, q27, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q28, q29, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q30, q31, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]


	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		q22, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v24.d[1]
	prfm	PLDL1KEEP, [x11]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x11, x12]
	fmla	v12.2d, v20.2d, v25.d[0]
	prfm	PLDL1KEEP, [x11, x14]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	prfm	PLDL1KEEP, [x11, x15]
	fmla	v14.2d, v20.2d, v25.d[1]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldr		q20, [x9, #32]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	sub		w8, w8, #4
	fmla	v14.2d, v22.2d, v27.d[1]

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldr		q22, [x9, #32]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v29.d[1]
	fmla	v7.2d, v17.2d, v29.d[1]
	ldp		q16, q17, [x9, #0]
	fmla	v14.2d, v20.2d, v29.d[1]
	ldr		q20, [x9, #32]

	// unroll 3
	fmla	v0.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v8.2d, v22.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	ldp		q24, q25, [x11, #(0*8)]
	fmla	v2.2d, v18.2d, v30.d[1]
	fmla	v3.2d, v19.2d, v30.d[1]
	add		x11, x11, x12
	fmla	v10.2d, v22.2d, v30.d[1]
	ldp		q26, q27, [x11, #(0*8)]
	fmla	v4.2d, v18.2d, v31.d[0]
	fmla	v5.2d, v19.2d, v31.d[0]
	add		x11, x11, x12
	fmla	v12.2d, v22.2d, v31.d[0]
	ldp		q28, q29, [x11, #(0*8)]
	fmla	v6.2d, v18.2d, v31.d[1]
	fmla	v7.2d, v19.2d, v31.d[1]
	add		x11, x11, x12
	fmla	v14.2d, v22.2d, v31.d[1]
	ldp		q30, q31, [x11, #(0*8)]
	add		x11, x11, x12

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		q22, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v10.2d, v20.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x13, #128]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x9, #192]
	fmla	v12.2d, v20.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x13, #192]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x11, #128]
	fmla	v14.2d, v20.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x11, #192]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldr		q20, [x9, #32]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v12.2d, v22.2d, v27.d[0]
//	add		x11, x11, #128
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	sub		w8, w8, #4
	fmla	v14.2d, v22.2d, v27.d[1]

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldr		q22, [x9, #32]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v6.2d, v16.2d, v29.d[1]
	fmla	v7.2d, v17.2d, v29.d[1]
	fmla	v14.2d, v20.2d, v29.d[1]

	// unroll 3
//	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldr		q20, [x9, #32]
	fmla	v8.2d, v22.2d, v30.d[0]
//	add		x9, x9, x10
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v3.2d, v19.2d, v30.d[1]
	fmla	v2.2d, v18.2d, v30.d[1]
	fmla	v10.2d, v22.2d, v30.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v31.d[0]
	fmla	v5.2d, v19.2d, v31.d[0]
	fmla	v12.2d, v22.2d, v31.d[0]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
	fmla	v6.2d, v18.2d, v31.d[1]
	fmla	v7.2d, v19.2d, v31.d[1]
	fmla	v14.2d, v22.2d, v31.d[1]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

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
	ldp		q20, q21, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10
	ldp		q28, q29, [x11]
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	fmla	v6.2d, v20.2d, v29.d[1]
	fmla	v7.2d, v21.2d, v29.d[1]
	fmla	v8.2d, v22.2d, v28.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v22.2d, v28.d[1]
	cmp		w8, #0
	fmla	v12.2d, v22.2d, v29.d[0]
	fmla	v14.2d, v22.2d, v29.d[1]

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_6x4_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NT_5X4_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_5x4_libcc)
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

	// main loop
1:
	
	// load 0 & 1
	ldp		q24, q25, [x11]
	add		x11, x11, x12
	ldp		q26, q27, [x11]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
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
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v14.2d, v20.2d, v25.d[1]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v14.2d, v22.2d, v27.d[1]

	// load 2 & 3
	ldp		q24, q25, [x11]
	add		x11, x11, x12
	ldp		q26, q27, [x11]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v14.2d, v20.2d, v25.d[1]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v14.2d, v22.2d, v27.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldp		q24, q25, [x11]
	add		x11, x11, x12
	ldp		q26, q27, [x11]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
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
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v14.2d, v20.2d, v25.d[1]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v14.2d, v22.2d, v27.d[1]

	// load 2 & 3
	ldp		q24, q25, [x11]
	add		x11, x11, x12
	ldp		q26, q27, [x11]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v14.2d, v20.2d, v25.d[1]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v14.2d, v22.2d, v27.d[1]

	sub		w8, w8, #4

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q24, q25, [x11]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v20.2d, v24.d[1]
	cmp		w8, #0
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v14.2d, v20.2d, v25.d[1]

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	add		x14, x12, x12
	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
	ldp		q24, q25, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q26, q27, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q28, q29, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q30, q31, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]


	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		d22, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v24.d[1]
	prfm	PLDL1KEEP, [x11]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x11, x12]
	fmla	v12.2d, v20.2d, v25.d[0]
	prfm	PLDL1KEEP, [x11, x14]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	prfm	PLDL1KEEP, [x11, x15]
	fmla	v14.2d, v20.2d, v25.d[1]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldr		d20, [x9, #32]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	sub		w8, w8, #4
	fmla	v14.2d, v22.2d, v27.d[1]

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldr		d22, [x9, #32]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v29.d[1]
	fmla	v7.2d, v17.2d, v29.d[1]
	ldp		q16, q17, [x9, #0]
	fmla	v14.2d, v20.2d, v29.d[1]
	ldr		d20, [x9, #32]

	// unroll 3
	fmla	v0.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v8.2d, v22.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	ldp		q24, q25, [x11, #(0*8)]
	fmla	v2.2d, v18.2d, v30.d[1]
	fmla	v3.2d, v19.2d, v30.d[1]
	add		x11, x11, x12
	fmla	v10.2d, v22.2d, v30.d[1]
	ldp		q26, q27, [x11, #(0*8)]
	fmla	v4.2d, v18.2d, v31.d[0]
	fmla	v5.2d, v19.2d, v31.d[0]
	add		x11, x11, x12
	fmla	v12.2d, v22.2d, v31.d[0]
	ldp		q28, q29, [x11, #(0*8)]
	fmla	v6.2d, v18.2d, v31.d[1]
	fmla	v7.2d, v19.2d, v31.d[1]
	add		x11, x11, x12
	fmla	v14.2d, v22.2d, v31.d[1]
	ldp		q30, q31, [x11, #(0*8)]
	add		x11, x11, x12

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		d22, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v10.2d, v20.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x13, #128]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x9, #192]
	fmla	v12.2d, v20.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x13, #192]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x11, #128]
	fmla	v14.2d, v20.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x11, #192]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldr		d20, [x9, #32]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v12.2d, v22.2d, v27.d[0]
//	add		x11, x11, #128
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	sub		w8, w8, #4
	fmla	v14.2d, v22.2d, v27.d[1]

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldr		d22, [x9, #32]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v6.2d, v16.2d, v29.d[1]
	fmla	v7.2d, v17.2d, v29.d[1]
	fmla	v14.2d, v20.2d, v29.d[1]

	// unroll 3
//	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldr		d20, [x9, #32]
	fmla	v8.2d, v22.2d, v30.d[0]
//	add		x9, x9, x10
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v3.2d, v19.2d, v30.d[1]
	fmla	v2.2d, v18.2d, v30.d[1]
	fmla	v10.2d, v22.2d, v30.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v31.d[0]
	fmla	v5.2d, v19.2d, v31.d[0]
	fmla	v12.2d, v22.2d, v31.d[0]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
	fmla	v6.2d, v18.2d, v31.d[1]
	fmla	v7.2d, v19.2d, v31.d[1]
	fmla	v14.2d, v22.2d, v31.d[1]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

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
	ldp		q20, q21, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10
	ldp		q28, q29, [x11]
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	fmla	v6.2d, v20.2d, v29.d[1]
	fmla	v7.2d, v21.2d, v29.d[1]
	fmla	v8.2d, v22.2d, v28.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v22.2d, v28.d[1]
	cmp		w8, #0
	fmla	v12.2d, v22.2d, v29.d[0]
	fmla	v14.2d, v22.2d, v29.d[1]

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_5x4_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NT_8X3_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_8x3_libcc)
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

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	ldr		d27, [x11, #16]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]

	// load 2 & 3
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	ldr		d27, [x11, #16]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	ldr		d27, [x11, #16]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]

	// load 2 & 3
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	ldr		d27, [x11, #16]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]

	sub		w8, w8, #4

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	cmp		w8, #0
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	add		x14, x12, x12
	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
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
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]


	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q22, q23, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	prfm	PLDL1KEEP, [x11]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x11, x12]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldp		q20, q21, [x9, #32]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldp		q22, q23, [x9, #32]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v28.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v11.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v13.2d, v21.2d, v29.d[0]
	cmp		w8, #4
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]

	// unroll 3
	fmla	v0.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v8.2d, v22.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v9.2d, v23.2d, v30.d[0]
	ldr		q24, [x11, #0]
	fmla	v2.2d, v18.2d, v30.d[1]
	ldr		d25, [x11, #16]
	fmla	v3.2d, v19.2d, v30.d[1]
	add		x11, x11, x12
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
	ldr		q26, [x11, #0]
	fmla	v4.2d, v18.2d, v31.d[0]
	ldr		d27, [x11, #16]
	fmla	v5.2d, v19.2d, v31.d[0]
	add		x11, x11, x12
	fmla	v12.2d, v22.2d, v31.d[0]
	fmla	v13.2d, v23.2d, v31.d[0]
	ldr		q28, [x11, #0]
	ldr		d29, [x11, #16]
	add		x11, x11, x12
	ldr		q30, [x11, #0]
	ldr		d31, [x11, #16]
	add		x11, x11, x12

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q22, q23, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x13, #128]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x9, #192]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x13, #192]
//	prfm	PLDL1KEEP, [x11, #128]
//	prfm	PLDL1KEEP, [x11, #192]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldp		q20, q21, [x9, #32]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]
//	add		x11, x11, #128
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldp		q22, q23, [x9, #32]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v28.d[0]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v11.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v13.2d, v21.2d, v29.d[0]

	// unroll 3
//	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldp		q20, q21, [x9, #32]
	fmla	v8.2d, v22.2d, v30.d[0]
//	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v30.d[0]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v3.2d, v19.2d, v30.d[1]
	fmla	v2.2d, v18.2d, v30.d[1]
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v31.d[0]
	fmla	v5.2d, v19.2d, v31.d[0]
	fmla	v12.2d, v22.2d, v31.d[0]
	fmla	v13.2d, v23.2d, v31.d[0]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

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
	ldp		q20, q21, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10
	ldr		q28, [x11, #0]
	ldr		d29, [x11, #16]
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	fmla	v8.2d, v22.2d, v28.d[0]
	fmla	v9.2d, v23.2d, v28.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v22.2d, v28.d[1]
	fmla	v11.2d, v23.2d, v28.d[1]
	cmp		w8, #0
	fmla	v12.2d, v22.2d, v29.d[0]
	fmla	v13.2d, v23.2d, v29.d[0]

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_8x3_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NT_7X3_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_7x3_libcc)
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

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	ldr		d27, [x11, #16]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]

	// load 2 & 3
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	ldr		d27, [x11, #16]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	ldr		d27, [x11, #16]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]

	// load 2 & 3
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	ldr		d27, [x11, #16]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]

	sub		w8, w8, #4

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	cmp		w8, #0
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	add		x14, x12, x12
	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
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
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]


	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	prfm	PLDL1KEEP, [x11]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x11, x12]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v28.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v11.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v13.2d, v21.2d, v29.d[0]
	cmp		w8, #4
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]

	// unroll 3
	fmla	v0.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v8.2d, v22.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v9.2d, v23.2d, v30.d[0]
	ldr		q24, [x11, #0]
	fmla	v2.2d, v18.2d, v30.d[1]
	ldr		d25, [x11, #16]
	fmla	v3.2d, v19.2d, v30.d[1]
	add		x11, x11, x12
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
	ldr		q26, [x11, #0]
	fmla	v4.2d, v18.2d, v31.d[0]
	ldr		d27, [x11, #16]
	fmla	v5.2d, v19.2d, v31.d[0]
	add		x11, x11, x12
	fmla	v12.2d, v22.2d, v31.d[0]
	fmla	v13.2d, v23.2d, v31.d[0]
	ldr		q28, [x11, #0]
	ldr		d29, [x11, #16]
	add		x11, x11, x12
	ldr		q30, [x11, #0]
	ldr		d31, [x11, #16]
	add		x11, x11, x12

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x13, #128]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x9, #192]
	fmla	v12.2d, v20.2d, v25.d[0]
	fmla	v13.2d, v21.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x13, #192]
//	prfm	PLDL1KEEP, [x11, #128]
//	prfm	PLDL1KEEP, [x11, #192]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v12.2d, v22.2d, v27.d[0]
	fmla	v13.2d, v23.2d, v27.d[0]
//	add		x11, x11, #128
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v28.d[0]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v11.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v13.2d, v21.2d, v29.d[0]

	// unroll 3
//	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldr		q20, [x9, #32]
//	ldr		d21, [x9, #48]
	fmla	v8.2d, v22.2d, v30.d[0]
//	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v30.d[0]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v3.2d, v19.2d, v30.d[1]
	fmla	v2.2d, v18.2d, v30.d[1]
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v31.d[0]
	fmla	v5.2d, v19.2d, v31.d[0]
	fmla	v12.2d, v22.2d, v31.d[0]
	fmla	v13.2d, v23.2d, v31.d[0]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

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
	ldp		q20, q21, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10
	ldr		q28, [x11, #0]
	ldr		d29, [x11, #16]
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	fmla	v8.2d, v22.2d, v28.d[0]
	fmla	v9.2d, v23.2d, v28.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v22.2d, v28.d[1]
	fmla	v11.2d, v23.2d, v28.d[1]
	cmp		w8, #0
	fmla	v12.2d, v22.2d, v29.d[0]
	fmla	v13.2d, v23.2d, v29.d[0]

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_7x3_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NT_6X3_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_6x3_libcc)
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

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	ldr		d27, [x11, #16]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]

	// load 2 & 3
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	ldr		d27, [x11, #16]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	ldr		d27, [x11, #16]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]

	// load 2 & 3
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	ldr		d27, [x11, #16]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]

	sub		w8, w8, #4

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v20.2d, v24.d[1]
	cmp		w8, #0
	fmla	v12.2d, v20.2d, v25.d[0]

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	add		x14, x12, x12
	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
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
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]


	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		q22, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v24.d[1]
	prfm	PLDL1KEEP, [x11]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x11, x12]
	fmla	v12.2d, v20.2d, v25.d[0]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldr		q20, [x9, #32]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v12.2d, v22.2d, v27.d[0]
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldr		q22, [x9, #32]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]
	cmp		w8, #4
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]

	// unroll 3
	fmla	v0.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v8.2d, v22.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	ldr		q24, [x11, #0]
	fmla	v2.2d, v18.2d, v30.d[1]
	ldr		d25, [x11, #16]
	fmla	v3.2d, v19.2d, v30.d[1]
	add		x11, x11, x12
	fmla	v10.2d, v22.2d, v30.d[1]
	ldr		q26, [x11, #0]
	fmla	v4.2d, v18.2d, v31.d[0]
	ldr		d27, [x11, #16]
	fmla	v5.2d, v19.2d, v31.d[0]
	add		x11, x11, x12
	fmla	v12.2d, v22.2d, v31.d[0]
	ldr		q28, [x11, #0]
	ldr		d29, [x11, #16]
	add		x11, x11, x12
	ldr		q30, [x11, #0]
	ldr		d31, [x11, #16]
	add		x11, x11, x12

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		q22, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v10.2d, v20.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x13, #128]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x9, #192]
	fmla	v12.2d, v20.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x13, #192]
//	prfm	PLDL1KEEP, [x11, #128]
//	prfm	PLDL1KEEP, [x11, #192]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldr		q20, [x9, #32]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v12.2d, v22.2d, v27.d[0]
//	add		x11, x11, #128
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldr		q22, [x9, #32]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]

	// unroll 3
//	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldr		q20, [x9, #32]
	fmla	v8.2d, v22.2d, v30.d[0]
//	add		x9, x9, x10
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v3.2d, v19.2d, v30.d[1]
	fmla	v2.2d, v18.2d, v30.d[1]
	fmla	v10.2d, v22.2d, v30.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v31.d[0]
	fmla	v5.2d, v19.2d, v31.d[0]
	fmla	v12.2d, v22.2d, v31.d[0]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

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
	ldp		q20, q21, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10
	ldr		q28, [x11, #0]
	ldr		d29, [x11, #16]
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	fmla	v8.2d, v22.2d, v28.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v22.2d, v28.d[1]
	cmp		w8, #0
	fmla	v12.2d, v22.2d, v29.d[0]

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_6x3_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NT_5X3_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_5x3_libcc)
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

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	ldr		d27, [x11, #16]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]

	// load 2 & 3
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	ldr		d27, [x11, #16]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	ldr		d27, [x11, #16]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]

	// load 2 & 3
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	ldr		d27, [x11, #16]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v12.2d, v20.2d, v25.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v27.d[0]

	sub		w8, w8, #4

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v20.2d, v24.d[1]
	cmp		w8, #0
	fmla	v12.2d, v20.2d, v25.d[0]

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	add		x14, x12, x12
	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
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
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]


	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		d22, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v24.d[1]
	prfm	PLDL1KEEP, [x11]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x11, x12]
	fmla	v12.2d, v20.2d, v25.d[0]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldr		d20, [x9, #32]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v12.2d, v22.2d, v27.d[0]
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldr		d22, [x9, #32]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]
	cmp		w8, #4
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]

	// unroll 3
	fmla	v0.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v8.2d, v22.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	ldr		q24, [x11, #0]
	fmla	v2.2d, v18.2d, v30.d[1]
	ldr		d25, [x11, #16]
	fmla	v3.2d, v19.2d, v30.d[1]
	add		x11, x11, x12
	fmla	v10.2d, v22.2d, v30.d[1]
	ldr		q26, [x11, #0]
	fmla	v4.2d, v18.2d, v31.d[0]
	ldr		d27, [x11, #16]
	fmla	v5.2d, v19.2d, v31.d[0]
	add		x11, x11, x12
	fmla	v12.2d, v22.2d, v31.d[0]
	ldr		q28, [x11, #0]
	ldr		d29, [x11, #16]
	add		x11, x11, x12
	ldr		q30, [x11, #0]
	ldr		d31, [x11, #16]
	add		x11, x11, x12

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		d22, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v10.2d, v20.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x13, #128]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x9, #192]
	fmla	v12.2d, v20.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x13, #192]
//	prfm	PLDL1KEEP, [x11, #128]
//	prfm	PLDL1KEEP, [x11, #192]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldr		d20, [x9, #32]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v12.2d, v22.2d, v27.d[0]
//	add		x11, x11, #128
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldr		d22, [x9, #32]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]

	// unroll 3
//	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldr		d20, [x9, #32]
	fmla	v8.2d, v22.2d, v30.d[0]
//	add		x9, x9, x10
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v3.2d, v19.2d, v30.d[1]
	fmla	v2.2d, v18.2d, v30.d[1]
	fmla	v10.2d, v22.2d, v30.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v31.d[0]
	fmla	v5.2d, v19.2d, v31.d[0]
	fmla	v12.2d, v22.2d, v31.d[0]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

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
	ldp		q20, q21, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10
	ldr		q28, [x11, #0]
	ldr		d29, [x11, #16]
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	fmla	v8.2d, v22.2d, v28.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v22.2d, v28.d[1]
	cmp		w8, #0
	fmla	v12.2d, v22.2d, v29.d[0]

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_5x3_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NT_8X2_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_8x2_libcc)
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

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]

	// load 2 & 3
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]

	// load 2 & 3
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]

	sub		w8, w8, #4

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldr		q24, [x11, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	add		x14, x12, x12
	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldr		q28, [x11, #0]
	add		x11, x11, x12
	ldr		q30, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]


	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q22, q23, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldp		q20, q21, [x9, #32]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldp		q22, q23, [x9, #32]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v28.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v11.2d, v21.2d, v28.d[1]
	cmp		w8, #4
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]

	// unroll 3
	fmla	v0.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v8.2d, v22.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v9.2d, v23.2d, v30.d[0]
	ldr		q24, [x11, #0]
	fmla	v2.2d, v18.2d, v30.d[1]
	fmla	v3.2d, v19.2d, v30.d[1]
	add		x11, x11, x12
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldr		q28, [x11, #0]
	add		x11, x11, x12
	ldr		q30, [x11, #0]
	add		x11, x11, x12

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q22, q23, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x13, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x13, #192]
//	prfm	PLDL1KEEP, [x11, #128]
//	prfm	PLDL1KEEP, [x11, #192]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldp		q20, q21, [x9, #32]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
//	add		x11, x11, #128
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldp		q22, q23, [x9, #32]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v28.d[0]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v11.2d, v21.2d, v28.d[1]

	// unroll 3
//	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldp		q20, q21, [x9, #32]
	fmla	v8.2d, v22.2d, v30.d[0]
//	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v30.d[0]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v3.2d, v19.2d, v30.d[1]
	fmla	v2.2d, v18.2d, v30.d[1]
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

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
	ldp		q20, q21, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10
	ldr		q28, [x11, #0]
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	fmla	v8.2d, v22.2d, v28.d[0]
	fmla	v9.2d, v23.2d, v28.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v22.2d, v28.d[1]
	fmla	v11.2d, v23.2d, v28.d[1]
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_8x2_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NT_7X2_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_7x2_libcc)
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

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]

	// load 2 & 3
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]

	// load 2 & 3
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]

	sub		w8, w8, #4

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldr		q24, [x11, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	add		x14, x12, x12
	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldr		q28, [x11, #0]
	add		x11, x11, x12
	ldr		q30, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]


	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v28.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v11.2d, v21.2d, v28.d[1]
	cmp		w8, #4
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]

	// unroll 3
	fmla	v0.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v8.2d, v22.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v9.2d, v23.2d, v30.d[0]
	ldr		q24, [x11, #0]
	fmla	v2.2d, v18.2d, v30.d[1]
	fmla	v3.2d, v19.2d, v30.d[1]
	add		x11, x11, x12
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldr		q28, [x11, #0]
	add		x11, x11, x12
	ldr		q30, [x11, #0]
	add		x11, x11, x12

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v10.2d, v20.2d, v24.d[1]
	fmla	v11.2d, v21.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x13, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x13, #192]
//	prfm	PLDL1KEEP, [x11, #128]
//	prfm	PLDL1KEEP, [x11, #192]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
//	add		x11, x11, #128
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v28.d[0]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	fmla	v10.2d, v20.2d, v28.d[1]
	fmla	v11.2d, v21.2d, v28.d[1]

	// unroll 3
//	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldr		q20, [x9, #32]
//	ldr		d21, [x9, #48]
	fmla	v8.2d, v22.2d, v30.d[0]
//	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v30.d[0]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v3.2d, v19.2d, v30.d[1]
	fmla	v2.2d, v18.2d, v30.d[1]
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

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
	ldp		q20, q21, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10
	ldr		q28, [x11, #0]
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	fmla	v8.2d, v22.2d, v28.d[0]
	fmla	v9.2d, v23.2d, v28.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v22.2d, v28.d[1]
	fmla	v11.2d, v23.2d, v28.d[1]
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_7x2_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NT_6X2_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_6x2_libcc)
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

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]

	// load 2 & 3
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]

	// load 2 & 3
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]

	sub		w8, w8, #4

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldr		q24, [x11, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v20.2d, v24.d[1]
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	add		x14, x12, x12
	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldr		q28, [x11, #0]
	add		x11, x11, x12
	ldr		q30, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]


	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		q22, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v24.d[1]
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldr		q20, [x9, #32]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v22.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldr		q22, [x9, #32]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v28.d[1]
	cmp		w8, #4
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]

	// unroll 3
	fmla	v0.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v8.2d, v22.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	ldr		q24, [x11, #0]
	fmla	v2.2d, v18.2d, v30.d[1]
	fmla	v3.2d, v19.2d, v30.d[1]
	add		x11, x11, x12
	fmla	v10.2d, v22.2d, v30.d[1]
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldr		q28, [x11, #0]
	add		x11, x11, x12
	ldr		q30, [x11, #0]
	add		x11, x11, x12

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		q22, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v10.2d, v20.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x13, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x13, #192]
//	prfm	PLDL1KEEP, [x11, #128]
//	prfm	PLDL1KEEP, [x11, #192]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldr		q20, [x9, #32]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
//	add		x11, x11, #128
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldr		q22, [x9, #32]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	fmla	v10.2d, v20.2d, v28.d[1]

	// unroll 3
//	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldr		q20, [x9, #32]
	fmla	v8.2d, v22.2d, v30.d[0]
//	add		x9, x9, x10
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v3.2d, v19.2d, v30.d[1]
	fmla	v2.2d, v18.2d, v30.d[1]
	fmla	v10.2d, v22.2d, v30.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

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
	ldp		q20, q21, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10
	ldr		q28, [x11, #0]
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	fmla	v8.2d, v22.2d, v28.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v22.2d, v28.d[1]
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_6x2_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NT_5X2_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_5x2_libcc)
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

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]

	// load 2 & 3
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]

	// load 2 & 3
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v10.2d, v20.2d, v24.d[1]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v10.2d, v22.2d, v26.d[1]

	sub		w8, w8, #4

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldr		q24, [x11, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v8.2d, v20.2d, v24.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v20.2d, v24.d[1]
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	add		x14, x12, x12
	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldr		q28, [x11, #0]
	add		x11, x11, x12
	ldr		q30, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]


	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		d22, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v24.d[1]
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldr		d20, [x9, #32]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v22.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldr		d22, [x9, #32]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v28.d[1]
	cmp		w8, #4
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]

	// unroll 3
	fmla	v0.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v8.2d, v22.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	ldr		q24, [x11, #0]
	fmla	v2.2d, v18.2d, v30.d[1]
	fmla	v3.2d, v19.2d, v30.d[1]
	add		x11, x11, x12
	fmla	v10.2d, v22.2d, v30.d[1]
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldr		q28, [x11, #0]
	add		x11, x11, x12
	ldr		q30, [x11, #0]
	add		x11, x11, x12

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		d22, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v10.2d, v20.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x13, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x13, #192]
//	prfm	PLDL1KEEP, [x11, #128]
//	prfm	PLDL1KEEP, [x11, #192]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldr		d20, [x9, #32]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
//	add		x11, x11, #128
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldr		d22, [x9, #32]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	fmla	v10.2d, v20.2d, v28.d[1]

	// unroll 3
//	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldr		d20, [x9, #32]
	fmla	v8.2d, v22.2d, v30.d[0]
//	add		x9, x9, x10
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v3.2d, v19.2d, v30.d[1]
	fmla	v2.2d, v18.2d, v30.d[1]
	fmla	v10.2d, v22.2d, v30.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

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
	ldp		q20, q21, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10
	ldr		q28, [x11, #0]
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	fmla	v8.2d, v22.2d, v28.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v22.2d, v28.d[1]
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_5x2_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NT_8X1_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_8x1_libcc)
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

	// main loop
1:
	
	// load 0 & 1
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]

	// load 2 & 3
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]

	// load 2 & 3
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]

	sub		w8, w8, #4

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldr		d24, [x11, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	add		x11, x11, x12
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	add		x14, x12, x12
	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldr		d28, [x11, #0]
	add		x11, x11, x12
	ldr		d30, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]


	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q22, q23, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldp		q20, q21, [x9, #32]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldp		q22, q23, [x9, #32]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v28.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]
	cmp		w8, #4
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]

	// unroll 3
	fmla	v0.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v8.2d, v22.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v9.2d, v23.2d, v30.d[0]
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldr		d28, [x11, #0]
	add		x11, x11, x12
	ldr		d30, [x11, #0]
	add		x11, x11, x12

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q22, q23, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x13, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x13, #192]
//	prfm	PLDL1KEEP, [x11, #128]
//	prfm	PLDL1KEEP, [x11, #192]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldp		q20, q21, [x9, #32]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v26.d[0]
//	add		x11, x11, #128
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldp		q22, q23, [x9, #32]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v28.d[0]

	// unroll 3
//	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldp		q20, q21, [x9, #32]
	fmla	v8.2d, v22.2d, v30.d[0]
//	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v30.d[0]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

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
	ldp		q20, q21, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10
	ldr		d28, [x11, #0]
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v8.2d, v22.2d, v28.d[0]
	fmla	v9.2d, v23.2d, v28.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_8x1_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NT_7X1_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_7x1_libcc)
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

	// main loop
1:
	
	// load 0 & 1
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]

	// load 2 & 3
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]

	// load 2 & 3
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]
	fmla	v9.2d, v23.2d, v26.d[0]

	sub		w8, w8, #4

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldr		d24, [x11, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	add		x11, x11, x12
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	add		x14, x12, x12
	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldr		d28, [x11, #0]
	add		x11, x11, x12
	ldr		d30, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]


	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v28.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]
	cmp		w8, #4
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]

	// unroll 3
	fmla	v0.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v8.2d, v22.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v9.2d, v23.2d, v30.d[0]
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldr		d28, [x11, #0]
	add		x11, x11, x12
	ldr		d30, [x11, #0]
	add		x11, x11, x12

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x13, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x13, #192]
//	prfm	PLDL1KEEP, [x11, #128]
//	prfm	PLDL1KEEP, [x11, #192]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v26.d[0]
//	add		x11, x11, #128
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v28.d[0]

	// unroll 3
//	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldr		q20, [x9, #32]
//	ldr		d21, [x9, #48]
	fmla	v8.2d, v22.2d, v30.d[0]
//	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v30.d[0]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

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
	ldp		q20, q21, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10
	ldr		d28, [x11, #0]
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v8.2d, v22.2d, v28.d[0]
	fmla	v9.2d, v23.2d, v28.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_7x1_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NT_6X1_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_6x1_libcc)
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

	// main loop
1:
	
	// load 0 & 1
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]

	// load 2 & 3
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]

	// load 2 & 3
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]

	sub		w8, w8, #4

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldr		d24, [x11, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	add		x11, x11, x12
	fmla	v8.2d, v20.2d, v24.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	add		x14, x12, x12
	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldr		d28, [x11, #0]
	add		x11, x11, x12
	ldr		d30, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]


	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		q22, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldr		q20, [x9, #32]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldr		q22, [x9, #32]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]
	cmp		w8, #4
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]

	// unroll 3
	fmla	v0.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v8.2d, v22.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldr		d28, [x11, #0]
	add		x11, x11, x12
	ldr		d30, [x11, #0]
	add		x11, x11, x12

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		q22, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x13, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x13, #192]
//	prfm	PLDL1KEEP, [x11, #128]
//	prfm	PLDL1KEEP, [x11, #192]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldr		q20, [x9, #32]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
//	add		x11, x11, #128
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldr		q22, [x9, #32]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10

	// unroll 3
//	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldr		q20, [x9, #32]
	fmla	v8.2d, v22.2d, v30.d[0]
//	add		x9, x9, x10
//	ldp		q24, q25, [x11, #(0*8+0*32)]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

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
	ldp		q20, q21, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10
	ldr		d28, [x11, #0]
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v8.2d, v22.2d, v28.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_6x1_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NT_5X1_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_5x1_libcc)
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

	// main loop
1:
	
	// load 0 & 1
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]

	// load 2 & 3
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]

	// load 2 & 3
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v8.2d, v22.2d, v26.d[0]

	sub		w8, w8, #4

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldr		d24, [x11, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	add		x11, x11, x12
	fmla	v8.2d, v20.2d, v24.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	add		x14, x12, x12
	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldr		d28, [x11, #0]
	add		x11, x11, x12
	ldr		d30, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]


	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		d22, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x11, x15]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldr		d20, [x9, #32]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldr		d22, [x9, #32]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]
	cmp		w8, #4
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]

	// unroll 3
	fmla	v0.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v8.2d, v22.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldr		d28, [x11, #0]
	add		x11, x11, x12
	ldr		d30, [x11, #0]
	add		x11, x11, x12

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		d22, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x13, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x13, #192]
//	prfm	PLDL1KEEP, [x11, #128]
//	prfm	PLDL1KEEP, [x11, #192]

	// unroll 1
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldr		d20, [x9, #32]
	fmla	v8.2d, v22.2d, v26.d[0]
	add		x9, x9, x10
//	add		x11, x11, #128
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	ldr		d22, [x9, #32]
	fmla	v8.2d, v20.2d, v28.d[0]
	add		x9, x9, x10

	// unroll 3
//	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldr		d20, [x9, #32]
	fmla	v8.2d, v22.2d, v30.d[0]
//	add		x9, x9, x10
//	ldp		q24, q25, [x11, #(0*8+0*32)]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

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
	ldp		q20, q21, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10
	ldr		d28, [x11, #0]
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v8.2d, v22.2d, v28.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_5x1_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NT_8X4_VS_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_8x4_vs_libcc)

	// save old return address
	sub sp, sp, #16
	str x30, [sp, #0]
#endif

	cmp		w13, #5
	bgt		90f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_5X4_LIBCC
#else
	CALL(inner_kernel_gemm_add_nt_5x4_libcc)
#endif

	b		93f

90:

	cmp		w13, #6
	bgt		91f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_6X4_LIBCC
#else
	CALL(inner_kernel_gemm_add_nt_6x4_libcc)
#endif
	
	b		93f

91:

	cmp		w13, #7
	bgt		92f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_7X4_LIBCC
#else
	CALL(inner_kernel_gemm_add_nt_7x4_libcc)
#endif
	
	b		93f

92:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIBCC
#else
	CALL(inner_kernel_gemm_add_nt_8x4_libcc)
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

	FUN_END(inner_kernel_gemm_add_nt_8x4_vs_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NT_8X3_VS_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_8x3_vs_libcc)

	// save old return address
	sub sp, sp, #16
	str x30, [sp, #0]
#endif

	cmp		w13, #5
	bgt		90f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_5X3_LIBCC
#else
	CALL(inner_kernel_gemm_add_nt_5x3_libcc)
#endif

	b		93f

90:

	cmp		w13, #6
	bgt		91f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_6X3_LIBCC
#else
	CALL(inner_kernel_gemm_add_nt_6x3_libcc)
#endif
	
	b		93f

91:

	cmp		w13, #7
	bgt		92f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_7X3_LIBCC
#else
	CALL(inner_kernel_gemm_add_nt_7x3_libcc)
#endif
	
	b		93f

92:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X3_LIBCC
#else
	CALL(inner_kernel_gemm_add_nt_8x3_libcc)
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

	FUN_END(inner_kernel_gemm_add_nt_8x3_vs_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NT_8X2_VS_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_8x2_vs_libcc)

	// save old return address
	sub sp, sp, #16
	str x30, [sp, #0]
#endif

	cmp		w13, #5
	bgt		90f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_5X2_LIBCC
#else
	CALL(inner_kernel_gemm_add_nt_5x2_libcc)
#endif

	b		93f

90:

	cmp		w13, #6
	bgt		91f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_6X2_LIBCC
#else
	CALL(inner_kernel_gemm_add_nt_6x2_libcc)
#endif
	
	b		93f

91:

	cmp		w13, #7
	bgt		92f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_7X2_LIBCC
#else
	CALL(inner_kernel_gemm_add_nt_7x2_libcc)
#endif
	
	b		93f

92:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X2_LIBCC
#else
	CALL(inner_kernel_gemm_add_nt_8x2_libcc)
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

	FUN_END(inner_kernel_gemm_add_nt_8x2_vs_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NT_8X1_VS_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_8x1_vs_libcc)

	// save old return address
	sub sp, sp, #16
	str x30, [sp, #0]
#endif

	cmp		w13, #5
	bgt		90f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_5X1_LIBCC
#else
	CALL(inner_kernel_gemm_add_nt_5x1_libcc)
#endif

	b		93f

90:

	cmp		w13, #6
	bgt		91f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_6X1_LIBCC
#else
	CALL(inner_kernel_gemm_add_nt_6x1_libcc)
#endif
	
	b		93f

91:

	cmp		w13, #7
	bgt		92f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_7X1_LIBCC
#else
	CALL(inner_kernel_gemm_add_nt_7x1_libcc)
#endif
	
	b		93f

92:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X1_LIBCC
#else
	CALL(inner_kernel_gemm_add_nt_8x1_libcc)
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

	FUN_END(inner_kernel_gemm_add_nt_8x1_vs_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- 8*lda
// x11   <- B
// x12   <- 8*ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_8X4_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_8x4_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	add		x14, x11, x12
	add		x15, x14, x12
	add		x16, x15, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldr		q27, [x16], #16
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v13.2d, v21.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v14.2d, v20.2d, v27.d[0]
	fmla	v15.2d, v21.2d, v27.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v13.2d, v23.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldr		q27, [x16], #16
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v13.2d, v21.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v14.2d, v20.2d, v27.d[0]
	fmla	v15.2d, v21.2d, v27.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v13.2d, v23.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldr		q27, [x16], #16
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v13.2d, v21.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v14.2d, v20.2d, v27.d[0]
	fmla	v15.2d, v21.2d, v27.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v13.2d, v23.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldr		q27, [x16], #16
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v13.2d, v21.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v14.2d, v20.2d, v27.d[0]
	fmla	v15.2d, v21.2d, v27.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v13.2d, v23.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8
	ldr		d30, [x15], #8
	ldr		d31, [x16], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v9.2d, v27.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	fmla	v11.2d, v27.2d, v29.d[0]
	add		x13, x13, #32
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	fmla	v12.2d, v26.2d, v30.d[0]
	fmla	v13.2d, v27.2d, v30.d[0]
	sub		w8, w8, #1
	fmla	v6.2d, v24.2d, v31.d[0]
	fmla	v7.2d, v25.2d, v31.d[0]
	cmp		w8, #0
	fmla	v14.2d, v26.2d, v31.d[0]
	fmla	v15.2d, v27.2d, v31.d[0]

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	//	add		x13, x9, x10

	add		x14, x11, x12
	add		x15, x14, x12
	add		x16, x15, x12

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x14], #32
	ldp		q28, q29, [x15], #32
	ldp		q30, q31, [x16], #32
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch
//	add		x14, x12, #64

//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	// main loop
1:
	
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q22, q23, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v26.d[0]
	fmla	v11.2d, v21.2d, v26.d[0]
	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x14, #32]
	fmla	v12.2d, v20.2d, v28.d[0]
	fmla	v13.2d, v21.2d, v28.d[0]
	prfm	PLDL1KEEP, [x15, #32]
	fmla	v6.2d, v16.2d, v30.d[0]
	fmla	v7.2d, v17.2d, v30.d[0]
	prfm	PLDL1KEEP, [x16, #32]
	fmla	v14.2d, v20.2d, v30.d[0]
	fmla	v15.2d, v21.2d, v30.d[0]

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldp		q20, q21, [x9, #32]
	fmla	v8.2d, v22.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	fmla	v12.2d, v22.2d, v28.d[1]
	fmla	v13.2d, v23.2d, v28.d[1]
	fmla	v6.2d, v18.2d, v30.d[1]
	fmla	v7.2d, v19.2d, v30.d[1]
	sub		w8, w8, #4
	fmla	v14.2d, v22.2d, v30.d[1]
	fmla	v15.2d, v23.2d, v30.d[1]

	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	ldp		q22, q23, [x9, #32]
	fmla	v8.2d, v20.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v25.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v27.d[0]
	fmla	v11.2d, v21.2d, v27.d[0]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v13.2d, v21.2d, v29.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v31.d[0]
	fmla	v7.2d, v17.2d, v31.d[0]
	fmla	v14.2d, v20.2d, v31.d[0]
	fmla	v15.2d, v21.2d, v31.d[0]

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	ldp		q20, q21, [x9, #32]
	fmla	v8.2d, v22.2d, v25.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v25.d[1]
	ldp		q24, q25, [x11], #32
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v10.2d, v22.2d, v27.d[1]
	fmla	v11.2d, v23.2d, v27.d[1]
	ldp		q26, q27, [x14], #32
	fmla	v4.2d, v18.2d, v29.d[1]
	fmla	v5.2d, v19.2d, v29.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v12.2d, v22.2d, v29.d[1]
	fmla	v13.2d, v23.2d, v29.d[1]
	ldp		q28, q29, [x15], #32
	fmla	v6.2d, v18.2d, v31.d[1]
	fmla	v7.2d, v19.2d, v31.d[1]
	fmla	v14.2d, v22.2d, v31.d[1]
	fmla	v15.2d, v23.2d, v31.d[1]
	ldp		q30, q31, [x16], #32

	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q22, q23, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x12]
	fmla	v10.2d, v20.2d, v26.d[0]
	fmla	v11.2d, v21.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x14]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v12.2d, v20.2d, v28.d[0]
	fmla	v13.2d, v21.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x13, #128]
	fmla	v6.2d, v16.2d, v30.d[0]
	fmla	v7.2d, v17.2d, v30.d[0]
//	prfm	PLDL1KEEP, [x9, #192]
	fmla	v14.2d, v20.2d, v30.d[0]
	fmla	v15.2d, v21.2d, v30.d[0]
//	prfm	PLDL1KEEP, [x13, #192]

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldp		q20, q21, [x9, #32]
	fmla	v8.2d, v22.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	fmla	v12.2d, v22.2d, v28.d[1]
	fmla	v13.2d, v23.2d, v28.d[1]
	fmla	v6.2d, v18.2d, v30.d[1]
	fmla	v7.2d, v19.2d, v30.d[1]
	sub		w8, w8, #4
	fmla	v14.2d, v22.2d, v30.d[1]
	fmla	v15.2d, v23.2d, v30.d[1]

	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	ldp		q22, q23, [x9, #32]
	fmla	v8.2d, v20.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v25.d[0]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	fmla	v10.2d, v20.2d, v27.d[0]
	fmla	v11.2d, v21.2d, v27.d[0]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v13.2d, v21.2d, v29.d[0]
	fmla	v6.2d, v16.2d, v31.d[0]
	fmla	v7.2d, v17.2d, v31.d[0]
	fmla	v14.2d, v20.2d, v31.d[0]
	fmla	v15.2d, v21.2d, v31.d[0]

//	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
//	ldp		q20, q21, [x9, #32]
	fmla	v8.2d, v22.2d, v25.d[1]
//	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v25.d[1]
//	ldp		q24, q25, [x11, #0]
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	fmla	v10.2d, v22.2d, v27.d[1]
	fmla	v11.2d, v23.2d, v27.d[1]
//	ldp		q26, q27, [x11, #32]
	fmla	v4.2d, v18.2d, v29.d[1]
	fmla	v5.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v22.2d, v29.d[1]
	fmla	v13.2d, v23.2d, v29.d[1]
//	ldp		q28, q29, [x11, #64]
	fmla	v6.2d, v18.2d, v31.d[1]
	fmla	v7.2d, v19.2d, v31.d[1]
	fmla	v14.2d, v22.2d, v31.d[1]
	fmla	v15.2d, v23.2d, v31.d[1]
//	ldp		q30, q31, [x11, #96]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x16, x16, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8
	ldr		d30, [x15], #8
	ldr		d31, [x16], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v9.2d, v27.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	fmla	v11.2d, v27.2d, v29.d[0]
	add		x13, x13, #32
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	fmla	v12.2d, v26.2d, v30.d[0]
	fmla	v13.2d, v27.2d, v30.d[0]
	sub		w8, w8, #1
	fmla	v6.2d, v24.2d, v31.d[0]
	fmla	v7.2d, v25.2d, v31.d[0]
	cmp		w8, #0
	fmla	v14.2d, v26.2d, v31.d[0]
	fmla	v15.2d, v27.2d, v31.d[0]

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_8x4_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- 8*lda
// x11   <- B
// x12   <- 8*ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_7X4_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_7x4_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	add		x14, x11, x12
	add		x15, x14, x12
	add		x16, x15, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldr		q27, [x16], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v13.2d, v21.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v14.2d, v20.2d, v27.d[0]
	fmla	v15.2d, v21.2d, v27.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v13.2d, v23.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldr		q27, [x16], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v13.2d, v21.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v14.2d, v20.2d, v27.d[0]
	fmla	v15.2d, v21.2d, v27.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v13.2d, v23.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldr		q27, [x16], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v13.2d, v21.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v14.2d, v20.2d, v27.d[0]
	fmla	v15.2d, v21.2d, v27.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v13.2d, v23.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldr		q27, [x16], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v13.2d, v21.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v14.2d, v20.2d, v27.d[0]
	fmla	v15.2d, v21.2d, v27.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v13.2d, v23.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v14.2d, v22.2d, v27.d[1]
	fmla	v15.2d, v23.2d, v27.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		q26, [x9, #32]
	ldr		d27, [x9, #32]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8
	ldr		d30, [x15], #8
	ldr		d31, [x16], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v9.2d, v27.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	fmla	v11.2d, v27.2d, v29.d[0]
	add		x13, x13, #32
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	fmla	v12.2d, v26.2d, v30.d[0]
	fmla	v13.2d, v27.2d, v30.d[0]
	sub		w8, w8, #1
	fmla	v6.2d, v24.2d, v31.d[0]
	fmla	v7.2d, v25.2d, v31.d[0]
	cmp		w8, #0
	fmla	v14.2d, v26.2d, v31.d[0]
	fmla	v15.2d, v27.2d, v31.d[0]

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	//	add		x13, x9, x10

	add		x14, x11, x12
	add		x15, x14, x12
	add		x16, x15, x12

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x14], #32
	ldp		q28, q29, [x15], #32
	ldp		q30, q31, [x16], #32
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch
//	add		x14, x12, #64

//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	// main loop
1:
	
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v26.d[0]
	fmla	v11.2d, v21.2d, v26.d[0]
	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x14, #32]
	fmla	v12.2d, v20.2d, v28.d[0]
	fmla	v13.2d, v21.2d, v28.d[0]
	prfm	PLDL1KEEP, [x15, #32]
	fmla	v6.2d, v16.2d, v30.d[0]
	fmla	v7.2d, v17.2d, v30.d[0]
	prfm	PLDL1KEEP, [x16, #32]
	fmla	v14.2d, v20.2d, v30.d[0]
	fmla	v15.2d, v21.2d, v30.d[0]

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	fmla	v8.2d, v22.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	fmla	v12.2d, v22.2d, v28.d[1]
	fmla	v13.2d, v23.2d, v28.d[1]
	fmla	v6.2d, v18.2d, v30.d[1]
	fmla	v7.2d, v19.2d, v30.d[1]
	sub		w8, w8, #4
	fmla	v14.2d, v22.2d, v30.d[1]
	fmla	v15.2d, v23.2d, v30.d[1]

	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	fmla	v8.2d, v20.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v25.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v27.d[0]
	fmla	v11.2d, v21.2d, v27.d[0]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v13.2d, v21.2d, v29.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v31.d[0]
	fmla	v7.2d, v17.2d, v31.d[0]
	fmla	v14.2d, v20.2d, v31.d[0]
	fmla	v15.2d, v21.2d, v31.d[0]

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	fmla	v8.2d, v22.2d, v25.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v25.d[1]
	ldp		q24, q25, [x11], #32
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v10.2d, v22.2d, v27.d[1]
	fmla	v11.2d, v23.2d, v27.d[1]
	ldp		q26, q27, [x14], #32
	fmla	v4.2d, v18.2d, v29.d[1]
	fmla	v5.2d, v19.2d, v29.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v12.2d, v22.2d, v29.d[1]
	fmla	v13.2d, v23.2d, v29.d[1]
	ldp		q28, q29, [x15], #32
	fmla	v6.2d, v18.2d, v31.d[1]
	fmla	v7.2d, v19.2d, v31.d[1]
	fmla	v14.2d, v22.2d, v31.d[1]
	fmla	v15.2d, v23.2d, v31.d[1]
	ldp		q30, q31, [x16], #32

	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x12]
	fmla	v10.2d, v20.2d, v26.d[0]
	fmla	v11.2d, v21.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x14]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v12.2d, v20.2d, v28.d[0]
	fmla	v13.2d, v21.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x13, #128]
	fmla	v6.2d, v16.2d, v30.d[0]
	fmla	v7.2d, v17.2d, v30.d[0]
//	prfm	PLDL1KEEP, [x9, #192]
	fmla	v14.2d, v20.2d, v30.d[0]
	fmla	v15.2d, v21.2d, v30.d[0]
//	prfm	PLDL1KEEP, [x13, #192]

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	fmla	v8.2d, v22.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	fmla	v12.2d, v22.2d, v28.d[1]
	fmla	v13.2d, v23.2d, v28.d[1]
	fmla	v6.2d, v18.2d, v30.d[1]
	fmla	v7.2d, v19.2d, v30.d[1]
	sub		w8, w8, #4
	fmla	v14.2d, v22.2d, v30.d[1]
	fmla	v15.2d, v23.2d, v30.d[1]

	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	fmla	v8.2d, v20.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v25.d[0]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	fmla	v10.2d, v20.2d, v27.d[0]
	fmla	v11.2d, v21.2d, v27.d[0]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v13.2d, v21.2d, v29.d[0]
	fmla	v6.2d, v16.2d, v31.d[0]
	fmla	v7.2d, v17.2d, v31.d[0]
	fmla	v14.2d, v20.2d, v31.d[0]
	fmla	v15.2d, v21.2d, v31.d[0]

//	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
//	ldr		q20, [x9, #32]
//	ldr		d21, [x9, #48]
	fmla	v8.2d, v22.2d, v25.d[1]
//	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v25.d[1]
//	ldp		q24, q25, [x11, #0]
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	fmla	v10.2d, v22.2d, v27.d[1]
	fmla	v11.2d, v23.2d, v27.d[1]
//	ldp		q26, q27, [x11, #32]
	fmla	v4.2d, v18.2d, v29.d[1]
	fmla	v5.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v22.2d, v29.d[1]
	fmla	v13.2d, v23.2d, v29.d[1]
//	ldp		q28, q29, [x11, #64]
	fmla	v6.2d, v18.2d, v31.d[1]
	fmla	v7.2d, v19.2d, v31.d[1]
	fmla	v14.2d, v22.2d, v31.d[1]
	fmla	v15.2d, v23.2d, v31.d[1]
//	ldp		q30, q31, [x11, #96]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x16, x16, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		q26, [x9, #32]
	ldr		d27, [x9, #48]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8
	ldr		d30, [x15], #8
	ldr		d31, [x16], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v9.2d, v27.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	fmla	v11.2d, v27.2d, v29.d[0]
	add		x13, x13, #32
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	fmla	v12.2d, v26.2d, v30.d[0]
	fmla	v13.2d, v27.2d, v30.d[0]
	sub		w8, w8, #1
	fmla	v6.2d, v24.2d, v31.d[0]
	fmla	v7.2d, v25.2d, v31.d[0]
	cmp		w8, #0
	fmla	v14.2d, v26.2d, v31.d[0]
	fmla	v15.2d, v27.2d, v31.d[0]

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_7x4_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- 8*lda
// x11   <- B
// x12   <- 8*ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_6X4_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_6x4_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	add		x14, x11, x12
	add		x15, x14, x12
	add		x16, x15, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldr		q27, [x16], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v14.2d, v20.2d, v27.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v14.2d, v22.2d, v27.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldr		q27, [x16], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v14.2d, v20.2d, v27.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v14.2d, v22.2d, v27.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldr		q27, [x16], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v14.2d, v20.2d, v27.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v14.2d, v22.2d, v27.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldr		q27, [x16], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v14.2d, v20.2d, v27.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v14.2d, v22.2d, v27.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		q26, [x9, #32]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8
	ldr		d30, [x15], #8
	ldr		d31, [x16], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	add		x13, x13, #32
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	fmla	v12.2d, v26.2d, v30.d[0]
	sub		w8, w8, #1
	fmla	v6.2d, v24.2d, v31.d[0]
	fmla	v7.2d, v25.2d, v31.d[0]
	cmp		w8, #0
	fmla	v14.2d, v26.2d, v31.d[0]

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	//	add		x13, x9, x10

	add		x14, x11, x12
	add		x15, x14, x12
	add		x16, x15, x12

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x14], #32
	ldp		q28, q29, [x15], #32
	ldp		q30, q31, [x16], #32
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch
//	add		x14, x12, #64

//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	// main loop
1:
	
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		q22, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v26.d[0]
	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x14, #32]
	fmla	v12.2d, v20.2d, v28.d[0]
	prfm	PLDL1KEEP, [x15, #32]
	fmla	v6.2d, v16.2d, v30.d[0]
	fmla	v7.2d, v17.2d, v30.d[0]
	prfm	PLDL1KEEP, [x16, #32]
	fmla	v14.2d, v20.2d, v30.d[0]

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldr		q20, [x9, #32]
	fmla	v8.2d, v22.2d, v24.d[1]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	fmla	v12.2d, v22.2d, v28.d[1]
	fmla	v6.2d, v18.2d, v30.d[1]
	fmla	v7.2d, v19.2d, v30.d[1]
	sub		w8, w8, #4
	fmla	v14.2d, v22.2d, v30.d[1]

	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	ldr		q22, [x9, #32]
	fmla	v8.2d, v20.2d, v25.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v27.d[0]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v31.d[0]
	fmla	v7.2d, v17.2d, v31.d[0]
	fmla	v14.2d, v20.2d, v31.d[0]

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	ldr		q20, [x9, #32]
	fmla	v8.2d, v22.2d, v25.d[1]
	add		x9, x9, x10
	ldp		q24, q25, [x11], #32
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v10.2d, v22.2d, v27.d[1]
	ldp		q26, q27, [x14], #32
	fmla	v4.2d, v18.2d, v29.d[1]
	fmla	v5.2d, v19.2d, v29.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v12.2d, v22.2d, v29.d[1]
	ldp		q28, q29, [x15], #32
	fmla	v6.2d, v18.2d, v31.d[1]
	fmla	v7.2d, v19.2d, v31.d[1]
	fmla	v14.2d, v22.2d, v31.d[1]
	ldp		q30, q31, [x16], #32

	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		q22, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x12]
	fmla	v10.2d, v20.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x14]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v12.2d, v20.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x13, #128]
	fmla	v6.2d, v16.2d, v30.d[0]
	fmla	v7.2d, v17.2d, v30.d[0]
//	prfm	PLDL1KEEP, [x9, #192]
	fmla	v14.2d, v20.2d, v30.d[0]
//	prfm	PLDL1KEEP, [x13, #192]

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldr		q20, [x9, #32]
	fmla	v8.2d, v22.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	fmla	v12.2d, v22.2d, v28.d[1]
	fmla	v6.2d, v18.2d, v30.d[1]
	fmla	v7.2d, v19.2d, v30.d[1]
	sub		w8, w8, #4
	fmla	v14.2d, v22.2d, v30.d[1]

	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	ldr		q22, [x9, #32]
	fmla	v8.2d, v20.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	fmla	v10.2d, v20.2d, v27.d[0]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v6.2d, v16.2d, v31.d[0]
	fmla	v7.2d, v17.2d, v31.d[0]
	fmla	v14.2d, v20.2d, v31.d[0]

//	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
//	ldr		q20, [x9, #32]
	fmla	v8.2d, v22.2d, v25.d[1]
//	add		x9, x9, x10
//	ldp		q24, q25, [x11, #0]
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	fmla	v10.2d, v22.2d, v27.d[1]
//	ldp		q26, q27, [x11, #32]
	fmla	v4.2d, v18.2d, v29.d[1]
	fmla	v5.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v22.2d, v29.d[1]
//	ldp		q28, q29, [x11, #64]
	fmla	v6.2d, v18.2d, v31.d[1]
	fmla	v7.2d, v19.2d, v31.d[1]
	fmla	v14.2d, v22.2d, v31.d[1]
//	ldp		q30, q31, [x11, #96]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x16, x16, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		q26, [x9, #32]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8
	ldr		d30, [x15], #8
	ldr		d31, [x16], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	add		x13, x13, #32
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	fmla	v12.2d, v26.2d, v30.d[0]
	sub		w8, w8, #1
	fmla	v6.2d, v24.2d, v31.d[0]
	fmla	v7.2d, v25.2d, v31.d[0]
	cmp		w8, #0
	fmla	v14.2d, v26.2d, v31.d[0]

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_6x4_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- 8*lda
// x11   <- B
// x12   <- 8*ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_5X4_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_5x4_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	add		x14, x11, x12
	add		x15, x14, x12
	add		x16, x15, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldr		q27, [x16], #16
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v14.2d, v20.2d, v27.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v14.2d, v22.2d, v27.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldr		q27, [x16], #16
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v14.2d, v20.2d, v27.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v14.2d, v22.2d, v27.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldr		q27, [x16], #16
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v14.2d, v20.2d, v27.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v14.2d, v22.2d, v27.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldr		q27, [x16], #16
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v14.2d, v20.2d, v27.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v14.2d, v22.2d, v27.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		d26, [x9, #32]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8
	ldr		d30, [x15], #8
	ldr		d31, [x16], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	add		x13, x13, #32
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	fmla	v12.2d, v26.2d, v30.d[0]
	sub		w8, w8, #1
	fmla	v6.2d, v24.2d, v31.d[0]
	fmla	v7.2d, v25.2d, v31.d[0]
	cmp		w8, #0
	fmla	v14.2d, v26.2d, v31.d[0]

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	//	add		x13, x9, x10

	add		x14, x11, x12
	add		x15, x14, x12
	add		x16, x15, x12

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x14], #32
	ldp		q28, q29, [x15], #32
	ldp		q30, q31, [x16], #32
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch
//	add		x14, x12, #64

//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	// main loop
1:
	
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		d22, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v26.d[0]
	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x14, #32]
	fmla	v12.2d, v20.2d, v28.d[0]
	prfm	PLDL1KEEP, [x15, #32]
	fmla	v6.2d, v16.2d, v30.d[0]
	fmla	v7.2d, v17.2d, v30.d[0]
	prfm	PLDL1KEEP, [x16, #32]
	fmla	v14.2d, v20.2d, v30.d[0]

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldr		d20, [x9, #32]
	fmla	v8.2d, v22.2d, v24.d[1]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	fmla	v12.2d, v22.2d, v28.d[1]
	fmla	v6.2d, v18.2d, v30.d[1]
	fmla	v7.2d, v19.2d, v30.d[1]
	sub		w8, w8, #4
	fmla	v14.2d, v22.2d, v30.d[1]

	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	ldr		d22, [x9, #32]
	fmla	v8.2d, v20.2d, v25.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v27.d[0]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v31.d[0]
	fmla	v7.2d, v17.2d, v31.d[0]
	fmla	v14.2d, v20.2d, v31.d[0]

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	ldr		d20, [x9, #32]
	fmla	v8.2d, v22.2d, v25.d[1]
	add		x9, x9, x10
	ldp		q24, q25, [x11], #32
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v10.2d, v22.2d, v27.d[1]
	ldp		q26, q27, [x14], #32
	fmla	v4.2d, v18.2d, v29.d[1]
	fmla	v5.2d, v19.2d, v29.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v12.2d, v22.2d, v29.d[1]
	ldp		q28, q29, [x15], #32
	fmla	v6.2d, v18.2d, v31.d[1]
	fmla	v7.2d, v19.2d, v31.d[1]
	fmla	v14.2d, v22.2d, v31.d[1]
	ldp		q30, q31, [x16], #32

	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		d22, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x12]
	fmla	v10.2d, v20.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x14]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v12.2d, v20.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x13, #128]
	fmla	v6.2d, v16.2d, v30.d[0]
	fmla	v7.2d, v17.2d, v30.d[0]
//	prfm	PLDL1KEEP, [x9, #192]
	fmla	v14.2d, v20.2d, v30.d[0]
//	prfm	PLDL1KEEP, [x13, #192]

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldr		d20, [x9, #32]
	fmla	v8.2d, v22.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	fmla	v12.2d, v22.2d, v28.d[1]
	fmla	v6.2d, v18.2d, v30.d[1]
	fmla	v7.2d, v19.2d, v30.d[1]
	sub		w8, w8, #4
	fmla	v14.2d, v22.2d, v30.d[1]

	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	ldr		d22, [x9, #32]
	fmla	v8.2d, v20.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	fmla	v10.2d, v20.2d, v27.d[0]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v6.2d, v16.2d, v31.d[0]
	fmla	v7.2d, v17.2d, v31.d[0]
	fmla	v14.2d, v20.2d, v31.d[0]

//	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
//	ldr		d20, [x9, #32]
	fmla	v8.2d, v22.2d, v25.d[1]
//	add		x9, x9, x10
//	ldp		q24, q25, [x11, #0]
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	fmla	v10.2d, v22.2d, v27.d[1]
//	ldp		q26, q27, [x11, #32]
	fmla	v4.2d, v18.2d, v29.d[1]
	fmla	v5.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v22.2d, v29.d[1]
//	ldp		q28, q29, [x11, #64]
	fmla	v6.2d, v18.2d, v31.d[1]
	fmla	v7.2d, v19.2d, v31.d[1]
	fmla	v14.2d, v22.2d, v31.d[1]
//	ldp		q30, q31, [x11, #96]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x16, x16, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		d26, [x9, #32]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8
	ldr		d30, [x15], #8
	ldr		d31, [x16], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	add		x13, x13, #32
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	fmla	v12.2d, v26.2d, v30.d[0]
	sub		w8, w8, #1
	fmla	v6.2d, v24.2d, v31.d[0]
	fmla	v7.2d, v25.2d, v31.d[0]
	cmp		w8, #0
	fmla	v14.2d, v26.2d, v31.d[0]

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_5x4_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- 8*lda
// x11   <- B
// x12   <- 8*ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_8X3_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_8x3_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	add		x14, x11, x12
	add		x15, x14, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v13.2d, v21.2d, v26.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v13.2d, v23.2d, v26.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v13.2d, v21.2d, v26.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v13.2d, v23.2d, v26.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v13.2d, v21.2d, v26.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v13.2d, v23.2d, v26.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v13.2d, v21.2d, v26.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v13.2d, v23.2d, v26.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8
	ldr		d30, [x15], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v9.2d, v27.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	fmla	v11.2d, v27.2d, v29.d[0]
	add		x13, x13, #32
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	fmla	v12.2d, v26.2d, v30.d[0]
	fmla	v13.2d, v27.2d, v30.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	//	add		x13, x9, x10

	add		x14, x11, x12
	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x14], #32
	ldp		q28, q29, [x15], #32
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch
//	add		x14, x12, #64

//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	// main loop
1:
	
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q22, q23, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v26.d[0]
	fmla	v11.2d, v21.2d, v26.d[0]
	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x14, #32]
	fmla	v12.2d, v20.2d, v28.d[0]
	fmla	v13.2d, v21.2d, v28.d[0]
	prfm	PLDL1KEEP, [x15, #32]

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldp		q20, q21, [x9, #32]
	fmla	v8.2d, v22.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	fmla	v12.2d, v22.2d, v28.d[1]
	fmla	v13.2d, v23.2d, v28.d[1]
	sub		w8, w8, #4

	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	ldp		q22, q23, [x9, #32]
	fmla	v8.2d, v20.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v25.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v27.d[0]
	fmla	v11.2d, v21.2d, v27.d[0]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v13.2d, v21.2d, v29.d[0]
	cmp		w8, #4

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	ldp		q20, q21, [x9, #32]
	fmla	v8.2d, v22.2d, v25.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v25.d[1]
	ldp		q24, q25, [x11], #32
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v10.2d, v22.2d, v27.d[1]
	fmla	v11.2d, v23.2d, v27.d[1]
	ldp		q26, q27, [x14], #32
	fmla	v4.2d, v18.2d, v29.d[1]
	fmla	v5.2d, v19.2d, v29.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v12.2d, v22.2d, v29.d[1]
	fmla	v13.2d, v23.2d, v29.d[1]
	ldp		q28, q29, [x15], #32

	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x12]
	fmla	v10.2d, v20.2d, v26.d[0]
	fmla	v11.2d, v21.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x14]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v12.2d, v20.2d, v28.d[0]
	fmla	v13.2d, v21.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x13, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x13, #192]

	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	fmla	v12.2d, v22.2d, v28.d[1]
	fmla	v13.2d, v23.2d, v28.d[1]
	sub		w8, w8, #4

	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v25.d[0]
	fmla	v9.2d, v21.2d, v25.d[0]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	fmla	v10.2d, v20.2d, v27.d[0]
	fmla	v11.2d, v21.2d, v27.d[0]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v13.2d, v21.2d, v29.d[0]

//	ldp		q16, q17, [x9, #0]
//	ldp		q20, q21, [x9, #32]
//	add		x9, x9, x10
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	fmla	v8.2d, v22.2d, v25.d[1]
	fmla	v9.2d, v23.2d, v25.d[1]
//	ldp		q24, q25, [x11, #0]
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	fmla	v10.2d, v22.2d, v27.d[1]
	fmla	v11.2d, v23.2d, v27.d[1]
//	ldp		q26, q27, [x11, #32]
	fmla	v4.2d, v18.2d, v29.d[1]
	fmla	v5.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v22.2d, v29.d[1]
	fmla	v13.2d, v23.2d, v29.d[1]
//	ldp		q28, q29, [x11, #64]
//	ldp		q30, q31, [x11, #96]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8
	ldr		d30, [x15], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v9.2d, v27.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	fmla	v11.2d, v27.2d, v29.d[0]
	add		x13, x13, #32
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	fmla	v12.2d, v26.2d, v30.d[0]
	fmla	v13.2d, v27.2d, v30.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_8x3_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- 8*lda
// x11   <- B
// x12   <- 8*ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_7X3_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_7x3_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	add		x14, x11, x12
	add		x15, x14, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v13.2d, v21.2d, v26.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v13.2d, v23.2d, v26.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v13.2d, v21.2d, v26.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v13.2d, v23.2d, v26.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v13.2d, v21.2d, v26.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v13.2d, v23.2d, v26.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]
	fmla	v13.2d, v21.2d, v26.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]
	fmla	v13.2d, v23.2d, v26.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		q26, [x9, #32]
	ldr		d27, [x9, #48]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8
	ldr		d30, [x15], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v9.2d, v27.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	fmla	v11.2d, v27.2d, v29.d[0]
	add		x13, x13, #32
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	fmla	v12.2d, v26.2d, v30.d[0]
	fmla	v13.2d, v27.2d, v30.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	//	add		x13, x9, x10

	add		x14, x11, x12
	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x14], #32
	ldp		q28, q29, [x15], #32
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch
//	add		x14, x12, #64

//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	// main loop
1:
	
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v26.d[0]
	fmla	v11.2d, v21.2d, v26.d[0]
	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x14, #32]
	fmla	v12.2d, v20.2d, v28.d[0]
	fmla	v13.2d, v21.2d, v28.d[0]
	prfm	PLDL1KEEP, [x15, #32]

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	fmla	v8.2d, v22.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	fmla	v12.2d, v22.2d, v28.d[1]
	fmla	v13.2d, v23.2d, v28.d[1]
	sub		w8, w8, #4

	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	fmla	v8.2d, v20.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v25.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v27.d[0]
	fmla	v11.2d, v21.2d, v27.d[0]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v13.2d, v21.2d, v29.d[0]
	cmp		w8, #4

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	fmla	v8.2d, v22.2d, v25.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v25.d[1]
	ldp		q24, q25, [x11], #32
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v10.2d, v22.2d, v27.d[1]
	fmla	v11.2d, v23.2d, v27.d[1]
	ldp		q26, q27, [x14], #32
	fmla	v4.2d, v18.2d, v29.d[1]
	fmla	v5.2d, v19.2d, v29.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v12.2d, v22.2d, v29.d[1]
	fmla	v13.2d, v23.2d, v29.d[1]
	ldp		q28, q29, [x15], #32

	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x12]
	fmla	v10.2d, v20.2d, v26.d[0]
	fmla	v11.2d, v21.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x14]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v12.2d, v20.2d, v28.d[0]
	fmla	v13.2d, v21.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x13, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x13, #192]

	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	fmla	v12.2d, v22.2d, v28.d[1]
	fmla	v13.2d, v23.2d, v28.d[1]
	sub		w8, w8, #4

	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v25.d[0]
	fmla	v9.2d, v21.2d, v25.d[0]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	fmla	v10.2d, v20.2d, v27.d[0]
	fmla	v11.2d, v21.2d, v27.d[0]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]
	fmla	v13.2d, v21.2d, v29.d[0]

//	ldp		q16, q17, [x9, #0]
//	ldr		q20, [x9, #32]
//	ldr		d21, [x9, #48]
//	add		x9, x9, x10
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	fmla	v8.2d, v22.2d, v25.d[1]
	fmla	v9.2d, v23.2d, v25.d[1]
//	ldp		q24, q25, [x11, #0]
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	fmla	v10.2d, v22.2d, v27.d[1]
	fmla	v11.2d, v23.2d, v27.d[1]
//	ldp		q26, q27, [x11, #32]
	fmla	v4.2d, v18.2d, v29.d[1]
	fmla	v5.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v22.2d, v29.d[1]
	fmla	v13.2d, v23.2d, v29.d[1]
//	ldp		q28, q29, [x11, #64]
//	ldp		q30, q31, [x11, #96]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		q26, [x9, #32]
	ldr		d27, [x9, #48]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8
	ldr		d30, [x15], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v9.2d, v27.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	fmla	v11.2d, v27.2d, v29.d[0]
	add		x13, x13, #32
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	fmla	v12.2d, v26.2d, v30.d[0]
	fmla	v13.2d, v27.2d, v30.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_7x3_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- 8*lda
// x11   <- B
// x12   <- 8*ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_6X3_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_6x3_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	add		x14, x11, x12
	add		x15, x14, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		q26, [x9, #32]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8
	ldr		d30, [x15], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	add		x13, x13, #32
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	fmla	v12.2d, v26.2d, v30.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	//	add		x13, x9, x10

	add		x14, x11, x12
	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x14], #32
	ldp		q28, q29, [x15], #32
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch
//	add		x14, x12, #64

//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	// main loop
1:
	
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		q22, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v26.d[0]
	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x14, #32]
	fmla	v12.2d, v20.2d, v28.d[0]
	prfm	PLDL1KEEP, [x15, #32]

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldr		q20, [x9, #32]
	fmla	v8.2d, v22.2d, v24.d[1]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	fmla	v12.2d, v22.2d, v28.d[1]
	sub		w8, w8, #4

	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	ldr		q22, [x9, #32]
	fmla	v8.2d, v20.2d, v25.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v27.d[0]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]
	cmp		w8, #4

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	ldr		q20, [x9, #32]
	fmla	v8.2d, v22.2d, v25.d[1]
	add		x9, x9, x10
	ldp		q24, q25, [x11], #32
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v10.2d, v22.2d, v27.d[1]
	ldp		q26, q27, [x14], #32
	fmla	v4.2d, v18.2d, v29.d[1]
	fmla	v5.2d, v19.2d, v29.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v12.2d, v22.2d, v29.d[1]
	ldp		q28, q29, [x15], #32

	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x12]
	fmla	v10.2d, v20.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x14]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v12.2d, v20.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x13, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x13, #192]

	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	fmla	v12.2d, v22.2d, v28.d[1]
	sub		w8, w8, #4

	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v25.d[0]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	fmla	v10.2d, v20.2d, v27.d[0]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]

//	ldp		q16, q17, [x9, #0]
//	ldr		q20, [x9, #32]
//	add		x9, x9, x10
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	fmla	v8.2d, v22.2d, v25.d[1]
//	ldp		q24, q25, [x11, #0]
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	fmla	v10.2d, v22.2d, v27.d[1]
//	ldp		q26, q27, [x11, #32]
	fmla	v4.2d, v18.2d, v29.d[1]
	fmla	v5.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v22.2d, v29.d[1]
//	ldp		q28, q29, [x11, #64]
//	ldp		q30, q31, [x11, #96]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		q26, [x9, #32]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8
	ldr		d30, [x15], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	add		x13, x13, #32
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	fmla	v12.2d, v26.2d, v30.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_6x3_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- 8*lda
// x11   <- B
// x12   <- 8*ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_5X3_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_5x3_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	add		x14, x11, x12
	add		x15, x14, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldr		q26, [x15], #16
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v12.2d, v20.2d, v26.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v12.2d, v22.2d, v26.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		d26, [x9, #32]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8
	ldr		d30, [x15], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	add		x13, x13, #32
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	fmla	v12.2d, v26.2d, v30.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	//	add		x13, x9, x10

	add		x14, x11, x12
	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x14], #32
	ldp		q28, q29, [x15], #32
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch
//	add		x14, x12, #64

//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	// main loop
1:
	
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		d22, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v26.d[0]
	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x14, #32]
	fmla	v12.2d, v20.2d, v28.d[0]
	prfm	PLDL1KEEP, [x15, #32]

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldr		d20, [x9, #32]
	fmla	v8.2d, v22.2d, v24.d[1]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	fmla	v12.2d, v22.2d, v28.d[1]
	sub		w8, w8, #4

	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	ldr		d22, [x9, #32]
	fmla	v8.2d, v20.2d, v25.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v27.d[0]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]
	cmp		w8, #4

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	ldr		d20, [x9, #32]
	fmla	v8.2d, v22.2d, v25.d[1]
	add		x9, x9, x10
	ldp		q24, q25, [x11], #32
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v10.2d, v22.2d, v27.d[1]
	ldp		q26, q27, [x14], #32
	fmla	v4.2d, v18.2d, v29.d[1]
	fmla	v5.2d, v19.2d, v29.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v12.2d, v22.2d, v29.d[1]
	ldp		q28, q29, [x15], #32

	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x12]
	fmla	v10.2d, v20.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x14]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v12.2d, v20.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x13, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x13, #192]

	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	fmla	v12.2d, v22.2d, v28.d[1]
	sub		w8, w8, #4

	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v25.d[0]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	fmla	v10.2d, v20.2d, v27.d[0]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v20.2d, v29.d[0]

//	ldp		q16, q17, [x9, #0]
//	ldr		d20, [x9, #32]
//	add		x9, x9, x10
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	fmla	v8.2d, v22.2d, v25.d[1]
//	ldp		q24, q25, [x11, #0]
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	fmla	v10.2d, v22.2d, v27.d[1]
//	ldp		q26, q27, [x11, #32]
	fmla	v4.2d, v18.2d, v29.d[1]
	fmla	v5.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v22.2d, v29.d[1]
//	ldp		q28, q29, [x11, #64]
//	ldp		q30, q31, [x11, #96]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		d26, [x9, #32]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8
	ldr		d30, [x15], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	add		x13, x13, #32
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	fmla	v12.2d, v26.2d, v30.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_5x3_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- 8*lda
// x11   <- B
// x12   <- 8*ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_8X2_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_8x2_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	add		x14, x11, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v9.2d, v27.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	fmla	v11.2d, v27.2d, v29.d[0]
	add		x13, x13, #32
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	//	add		x13, x9, x10

	add		x14, x11, x12

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x14], #32
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch
//	add		x14, x12, #64

//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	// main loop
1:
	
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q22, q23, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v26.d[0]
	fmla	v11.2d, v21.2d, v26.d[0]
	prfm	PLDL1KEEP, [x11, #32]
	prfm	PLDL1KEEP, [x14, #32]

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldp		q20, q21, [x9, #32]
	fmla	v8.2d, v22.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	sub		w8, w8, #4

	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	ldp		q22, q23, [x9, #32]
	fmla	v8.2d, v20.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v25.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v27.d[0]
	fmla	v11.2d, v21.2d, v27.d[0]
	cmp		w8, #4

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	ldp		q20, q21, [x9, #32]
	fmla	v8.2d, v22.2d, v25.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v25.d[1]
	ldp		q24, q25, [x11], #32
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v10.2d, v22.2d, v27.d[1]
	fmla	v11.2d, v23.2d, v27.d[1]
	ldp		q26, q27, [x14], #32
	prfm	PLDL1KEEP, [x9, x18]

	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x12]
	fmla	v10.2d, v20.2d, v26.d[0]
	fmla	v11.2d, v21.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x13, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x13, #192]

	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	sub		w8, w8, #4

	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v25.d[0]
	fmla	v9.2d, v21.2d, v25.d[0]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	fmla	v10.2d, v20.2d, v27.d[0]
	fmla	v11.2d, v21.2d, v27.d[0]

//	ldp		q16, q17, [x9, #0]
//	ldp		q20, q21, [x9, #32]
//	add		x9, x9, x10
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	fmla	v8.2d, v22.2d, v25.d[1]
	fmla	v9.2d, v23.2d, v25.d[1]
//	ldp		q24, q25, [x11, #0]
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	fmla	v10.2d, v22.2d, v27.d[1]
	fmla	v11.2d, v23.2d, v27.d[1]
//	ldp		q26, q27, [x11, #32]
//	ldp		q28, q29, [x11, #64]
//	ldp		q30, q31, [x11, #96]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x14, x14, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v9.2d, v27.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	fmla	v11.2d, v27.2d, v29.d[0]
	add		x13, x13, #32
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_8x2_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- 8*lda
// x11   <- B
// x12   <- 8*ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_7X2_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_7x2_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	add		x14, x11, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]
	fmla	v11.2d, v21.2d, v25.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]
	fmla	v11.2d, v23.2d, v25.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		q26, [x9, #32]
	ldr		d27, [x9, #48]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v9.2d, v27.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	fmla	v11.2d, v27.2d, v29.d[0]
	add		x13, x13, #32
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	//	add		x13, x9, x10

	add		x14, x11, x12

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x14], #32
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch
//	add		x14, x12, #64

//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	// main loop
1:
	
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v26.d[0]
	fmla	v11.2d, v21.2d, v26.d[0]
	prfm	PLDL1KEEP, [x11, #32]
	prfm	PLDL1KEEP, [x14, #32]

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	fmla	v8.2d, v22.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	sub		w8, w8, #4

	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	fmla	v8.2d, v20.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v25.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v27.d[0]
	fmla	v11.2d, v21.2d, v27.d[0]
	cmp		w8, #4

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	fmla	v8.2d, v22.2d, v25.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v25.d[1]
	ldp		q24, q25, [x11], #32
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v10.2d, v22.2d, v27.d[1]
	fmla	v11.2d, v23.2d, v27.d[1]
	ldp		q26, q27, [x14], #32
	prfm	PLDL1KEEP, [x9, x18]

	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x12]
	fmla	v10.2d, v20.2d, v26.d[0]
	fmla	v11.2d, v21.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x13, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x13, #192]

	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	fmla	v11.2d, v23.2d, v26.d[1]
	sub		w8, w8, #4

	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v25.d[0]
	fmla	v9.2d, v21.2d, v25.d[0]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	fmla	v10.2d, v20.2d, v27.d[0]
	fmla	v11.2d, v21.2d, v27.d[0]

//	ldp		q16, q17, [x9, #0]
//	ldr		q20, [x9, #32]
//	ldr		d21, [x9, #48]
//	add		x9, x9, x10
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	fmla	v8.2d, v22.2d, v25.d[1]
	fmla	v9.2d, v23.2d, v25.d[1]
//	ldp		q24, q25, [x11, #0]
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	fmla	v10.2d, v22.2d, v27.d[1]
	fmla	v11.2d, v23.2d, v27.d[1]
//	ldp		q26, q27, [x11, #32]
//	ldp		q28, q29, [x11, #64]
//	ldp		q30, q31, [x11, #96]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x14, x14, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		q26, [x9, #32]
	ldr		d27, [x9, #48]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v9.2d, v27.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	fmla	v11.2d, v27.2d, v29.d[0]
	add		x13, x13, #32
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_7x2_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- 8*lda
// x11   <- B
// x12   <- 8*ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_6X2_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_6x2_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	add		x14, x11, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		q26, [x9, #32]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	add		x13, x13, #32
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	//	add		x13, x9, x10

	add		x14, x11, x12

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x14], #32
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch
//	add		x14, x12, #64

//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	// main loop
1:
	
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		q22, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v26.d[0]
	prfm	PLDL1KEEP, [x11, #32]
	prfm	PLDL1KEEP, [x14, #32]

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldr		q20, [x9, #32]
	fmla	v8.2d, v22.2d, v24.d[1]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v22.2d, v26.d[1]
	sub		w8, w8, #4

	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	ldr		q22, [x9, #32]
	fmla	v8.2d, v20.2d, v25.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v27.d[0]
	cmp		w8, #4

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	ldr		q20, [x9, #32]
	fmla	v8.2d, v22.2d, v25.d[1]
	add		x9, x9, x10
	ldp		q24, q25, [x11], #32
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v10.2d, v22.2d, v27.d[1]
	ldp		q26, q27, [x14], #32
	prfm	PLDL1KEEP, [x9, x18]

	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x12]
	fmla	v10.2d, v20.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x13, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x13, #192]

	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	sub		w8, w8, #4

	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v25.d[0]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	fmla	v10.2d, v20.2d, v27.d[0]

//	ldp		q16, q17, [x9, #0]
//	ldr		q20, [x9, #32]
//	add		x9, x9, x10
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	fmla	v8.2d, v22.2d, v25.d[1]
//	ldp		q24, q25, [x11, #0]
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	fmla	v10.2d, v22.2d, v27.d[1]
//	ldp		q26, q27, [x11, #32]
//	ldp		q28, q29, [x11, #64]
//	ldp		q30, q31, [x11, #96]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x14, x14, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		q26, [x9, #32]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	add		x13, x13, #32
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_6x2_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- 8*lda
// x11   <- B
// x12   <- 8*ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_5X2_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_5x2_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	add		x14, x11, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x14], #16
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v10.2d, v20.2d, v25.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v10.2d, v22.2d, v25.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		d26, [x9, #32]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	add		x13, x13, #32
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	//	add		x13, x9, x10

	add		x14, x11, x12

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x14], #32
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch
//	add		x14, x12, #64

//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	// main loop
1:
	
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		d22, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v26.d[0]
	prfm	PLDL1KEEP, [x11, #32]
	prfm	PLDL1KEEP, [x14, #32]

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldr		d20, [x9, #32]
	fmla	v8.2d, v22.2d, v24.d[1]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v22.2d, v26.d[1]
	sub		w8, w8, #4

	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	ldr		d22, [x9, #32]
	fmla	v8.2d, v20.2d, v25.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	prfm	PLDL1KEEP, [x9, x18]
	fmla	v10.2d, v20.2d, v27.d[0]
	cmp		w8, #4

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	ldr		d20, [x9, #32]
	fmla	v8.2d, v22.2d, v25.d[1]
	add		x9, x9, x10
	ldp		q24, q25, [x11], #32
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	prfm	PLDL1KEEP, [x9, x17]
	fmla	v10.2d, v22.2d, v27.d[1]
	ldp		q26, q27, [x14], #32
	prfm	PLDL1KEEP, [x9, x18]

	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x12]
	fmla	v10.2d, v20.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x13, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x13, #192]

	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v10.2d, v22.2d, v26.d[1]
	sub		w8, w8, #4

	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v25.d[0]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	fmla	v10.2d, v20.2d, v27.d[0]

//	ldp		q16, q17, [x9, #0]
//	ldr		d20, [x9, #32]
//	add		x9, x9, x10
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	fmla	v8.2d, v22.2d, v25.d[1]
//	ldp		q24, q25, [x11, #0]
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
	fmla	v10.2d, v22.2d, v27.d[1]
//	ldp		q26, q27, [x11, #32]
//	ldp		q28, q29, [x11, #64]
//	ldp		q30, q31, [x11, #96]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x14, x14, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		d26, [x9, #32]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x14], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	add		x13, x13, #32
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_5x2_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- 8*lda
// x11   <- B
// x12   <- 8*ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_8X1_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_8x1_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	add		x9, x9, x10
	ldr		d28, [x11], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v9.2d, v27.2d, v28.d[0]
	add		x13, x13, #32
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	//	add		x13, x9, x10

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch
//	add		x14, x12, #64

//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	// main loop
1:
	
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q22, q23, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]
	prfm	PLDL1KEEP, [x11, #32]

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldp		q20, q21, [x9, #32]
	fmla	v8.2d, v22.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]
	sub		w8, w8, #4

	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	ldp		q22, q23, [x9, #32]
	fmla	v8.2d, v20.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v25.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]
	cmp		w8, #4

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	ldp		q20, q21, [x9, #32]
	fmla	v8.2d, v22.2d, v25.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v25.d[1]
	ldp		q24, q25, [x11], #32
	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x13, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x13, #192]

	ldp		q16, q17, [x9, #0]
	ldp		q20, q21, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	sub		w8, w8, #4

	ldp		q18, q19, [x9, #0]
	ldp		q22, q23, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v25.d[0]
	fmla	v9.2d, v21.2d, v25.d[0]

//	ldp		q16, q17, [x9, #0]
//	ldp		q20, q21, [x9, #32]
//	add		x9, x9, x10
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	fmla	v8.2d, v22.2d, v25.d[1]
	fmla	v9.2d, v23.2d, v25.d[1]
//	ldp		q24, q25, [x11, #0]
//	ldp		q26, q27, [x11, #32]
//	ldp		q28, q29, [x11, #64]
//	ldp		q30, q31, [x11, #96]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
//	sub		x14, x14, #32
//	sub		x15, x15, #32
//	sub		x16, x16, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	add		x9, x9, x10
	ldr		d28, [x11], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v9.2d, v27.2d, v28.d[0]
	add		x13, x13, #32
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_8x1_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- 8*lda
// x11   <- B
// x12   <- 8*ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_7X1_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_7x1_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		q26, [x9, #32]
	ldr		d27, [x9, #48]
	add		x9, x9, x10
	ldr		d28, [x11], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v9.2d, v27.2d, v28.d[0]
	add		x13, x13, #32
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	//	add		x13, x9, x10

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch
//	add		x14, x12, #64

//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	// main loop
1:
	
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]
	prfm	PLDL1KEEP, [x11, #32]

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	fmla	v8.2d, v22.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]
	sub		w8, w8, #4

	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	fmla	v8.2d, v20.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v21.2d, v25.d[0]
	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]
	cmp		w8, #4

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	fmla	v8.2d, v22.2d, v25.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v23.2d, v25.d[1]
	ldp		q24, q25, [x11], #32
	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
	fmla	v9.2d, v21.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x13, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x13, #192]

	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	ldr		d21, [x9, #48]
	add		x9, x9, x10
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	fmla	v9.2d, v23.2d, v24.d[1]
	sub		w8, w8, #4

	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	ldr		d23, [x9, #48]
	add		x9, x9, x10
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v25.d[0]
	fmla	v9.2d, v21.2d, v25.d[0]

//	ldp		q16, q17, [x9, #0]
//	ldr		q20, [x9, #32]
//	ldr		d21, [x9, #48]
//	add		x9, x9, x10
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	fmla	v8.2d, v22.2d, v25.d[1]
	fmla	v9.2d, v23.2d, v25.d[1]
//	ldp		q24, q25, [x11, #0]
//	ldp		q26, q27, [x11, #32]
//	ldp		q28, q29, [x11, #64]
//	ldp		q30, q31, [x11, #96]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
//	sub		x14, x14, #32
//	sub		x15, x15, #32
//	sub		x16, x16, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		q26, [x9, #32]
	ldr		d27, [x9, #48]
	add		x9, x9, x10
	ldr		d28, [x11], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v9.2d, v27.2d, v28.d[0]
	add		x13, x13, #32
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_7x1_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- 8*lda
// x11   <- B
// x12   <- 8*ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_6X1_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_6x1_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		q26, [x9, #32]
	add		x9, x9, x10
	ldr		d28, [x11], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	add		x13, x13, #32
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	//	add		x13, x9, x10

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch
//	add		x14, x12, #64

//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	// main loop
1:
	
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		q22, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]
	prfm	PLDL1KEEP, [x11, #32]

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldr		q20, [x9, #32]
	fmla	v8.2d, v22.2d, v24.d[1]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]
	sub		w8, w8, #4

	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	ldr		q22, [x9, #32]
	fmla	v8.2d, v20.2d, v25.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]
	cmp		w8, #4

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	ldr		q20, [x9, #32]
	fmla	v8.2d, v22.2d, v25.d[1]
	add		x9, x9, x10
	ldp		q24, q25, [x11], #32
	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x13, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x13, #192]

	ldp		q16, q17, [x9, #0]
	ldr		q20, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	sub		w8, w8, #4

	ldp		q18, q19, [x9, #0]
	ldr		q22, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v25.d[0]

//	ldp		q16, q17, [x9, #0]
//	ldr		q20, [x9, #32]
//	add		x9, x9, x10
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	fmla	v8.2d, v22.2d, v25.d[1]
//	ldp		q24, q25, [x11, #0]
//	ldp		q26, q27, [x11, #32]
//	ldp		q28, q29, [x11, #64]
//	ldp		q30, q31, [x11, #96]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
//	sub		x14, x14, #32
//	sub		x15, x15, #32
//	sub		x16, x16, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		q26, [x9, #32]
	add		x9, x9, x10
	ldr		d28, [x11], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	add		x13, x13, #32
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_6x1_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- 8*lda
// x11   <- B
// x12   <- 8*ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_5X1_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_5x1_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x13, x9, x10

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]

	// unroll 0
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]

	// load 2 & 3
	ldr		q24, [x11], #16
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		d26, [x9, #32]
	add		x9, x9, x10
	ldr		d28, [x11], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	add		x13, x13, #32
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	//	add		x13, x9, x10

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch
//	add		x14, x12, #64

//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #32]

	add		x17, x10, x10
	add		x18, x17, #32

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	add		x17, x17, x10
	add		x18, x18, x10

	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	// main loop
1:
	
	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldr		d22, [x9, #32]
	fmla	v8.2d, v20.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]
	prfm	PLDL1KEEP, [x11, #32]

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldr		d20, [x9, #32]
	fmla	v8.2d, v22.2d, v24.d[1]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]
	sub		w8, w8, #4

	ldp		q18, q19, [x9, #0]
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	ldr		d22, [x9, #32]
	fmla	v8.2d, v20.2d, v25.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]
	cmp		w8, #4

	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	ldr		d20, [x9, #32]
	fmla	v8.2d, v22.2d, v25.d[1]
	add		x9, x9, x10
	ldp		q24, q25, [x11], #32
	prfm	PLDL1KEEP, [x9, x17]
	prfm	PLDL1KEEP, [x9, x18]

	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v8.2d, v20.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x13, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x13, #192]

	ldp		q16, q17, [x9, #0]
	ldr		d20, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v8.2d, v22.2d, v24.d[1]
	sub		w8, w8, #4

	ldp		q18, q19, [x9, #0]
	ldr		d22, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	fmla	v8.2d, v20.2d, v25.d[0]

//	ldp		q16, q17, [x9, #0]
//	ldr		d20, [x9, #32]
//	add		x9, x9, x10
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
	fmla	v8.2d, v22.2d, v25.d[1]
//	ldp		q24, q25, [x11, #0]
//	ldp		q26, q27, [x11, #32]
//	ldp		q28, q29, [x11, #64]
//	ldp		q30, q31, [x11, #96]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
//	sub		x14, x14, #32
//	sub		x15, x15, #32
//	sub		x16, x16, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		d26, [x9, #32]
	add		x9, x9, x10
	ldr		d28, [x11], #8

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	add		x13, x13, #32
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_5x1_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_8X4_VS_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_8x4_vs_libcc)

	// save old return address
	sub sp, sp, #16
	str x30, [sp, #0]
#endif

	cmp		w13, #5
	bgt		90f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_5X4_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_5x4_libcc)
#endif

	b		93f

90:

	cmp		w13, #6
	bgt		91f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_6X4_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_6x4_libcc)
#endif
	
	b		93f

91:

	cmp		w13, #7
	bgt		92f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_7X4_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_7x4_libcc)
#endif
	
	b		93f

92:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X4_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_8x4_libcc)
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

	FUN_END(inner_kernel_gemm_add_nn_8x4_vs_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_8X3_VS_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_8x3_vs_libcc)

	// save old return address
	sub sp, sp, #16
	str x30, [sp, #0]
#endif

	cmp		w13, #5
	bgt		90f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_5X3_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_5x3_libcc)
#endif

	b		93f

90:

	cmp		w13, #6
	bgt		91f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_6X3_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_6x3_libcc)
#endif
	
	b		93f

91:

	cmp		w13, #7
	bgt		92f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_7X3_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_7x3_libcc)
#endif
	
	b		93f

92:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X3_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_8x3_libcc)
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

	FUN_END(inner_kernel_gemm_add_nn_8x3_vs_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_8X2_VS_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_8x2_vs_libcc)

	// save old return address
	sub sp, sp, #16
	str x30, [sp, #0]
#endif

	cmp		w13, #5
	bgt		90f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_5X2_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_5x2_libcc)
#endif

	b		93f

90:

	cmp		w13, #6
	bgt		91f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_6X2_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_6x2_libcc)
#endif
	
	b		93f

91:

	cmp		w13, #7
	bgt		92f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_7X2_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_7x2_libcc)
#endif
	
	b		93f

92:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X2_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_8x2_libcc)
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

	FUN_END(inner_kernel_gemm_add_nn_8x2_vs_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_8X1_VS_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_8x1_vs_libcc)

	// save old return address
	sub sp, sp, #16
	str x30, [sp, #0]
#endif

	cmp		w13, #5
	bgt		90f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_5X1_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_5x1_libcc)
#endif

	b		93f

90:

	cmp		w13, #6
	bgt		91f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_6X1_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_6x1_libcc)
#endif
	
	b		93f

91:

	cmp		w13, #7
	bgt		92f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_7X1_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_7x1_libcc)
#endif
	
	b		93f

92:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X1_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_8x1_libcc)
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

	FUN_END(inner_kernel_gemm_add_nn_8x1_vs_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NT_4X8_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_4x8_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x11, x11
	add		x13, x12, x11

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// main loop
1:
	
	// unroll 0
	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	add		x10, x10, x11
	ldp		q28, q29, [x10, #0] // B
	ldp		q30, q31, [x10, #32] // B
	add		x10, x10, x11

	ldp		q16, q17, [x9, #(0*8+0*32)]
	ldp		q18, q19, [x9, #(0*8+1*32)]
	ldp		q20, q21, [x9, #(0*8+2*32)]
	ldp		q22, q23, [x9, #(0*8+3*32)]
	add		x9, x9, #128

	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	fmla	v8.2d, v16.2d, v26.d[0]
	fmla	v9.2d, v17.2d, v26.d[0]
	fmla	v10.2d, v16.2d, v26.d[1]
	fmla	v11.2d, v17.2d, v26.d[1]
	fmla	v12.2d, v16.2d, v27.d[0]
	fmla	v13.2d, v17.2d, v27.d[0]
	fmla	v14.2d, v16.2d, v27.d[1]
	fmla	v15.2d, v17.2d, v27.d[1]

	// unroll 1
	fmla	v0.2d, v18.2d, v28.d[0]
	fmla	v1.2d, v19.2d, v28.d[0]
	fmla	v2.2d, v18.2d, v28.d[1]
	fmla	v3.2d, v19.2d, v28.d[1]
	fmla	v4.2d, v18.2d, v29.d[0]
	fmla	v5.2d, v19.2d, v29.d[0]
	fmla	v6.2d, v18.2d, v29.d[1]
	fmla	v7.2d, v19.2d, v29.d[1]
	fmla	v8.2d, v18.2d, v30.d[0]
	fmla	v9.2d, v19.2d, v30.d[0]
	fmla	v10.2d, v18.2d, v30.d[1]
	fmla	v11.2d, v19.2d, v30.d[1]
	fmla	v12.2d, v18.2d, v31.d[0]
	fmla	v13.2d, v19.2d, v31.d[0]
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]

	// unroll 2
	ldp		q24, q25, [x10, #0] // B
	ldp		q26, q27, [x10, #32] // B
	add		x10, x10, x11
	ldp		q28, q29, [x10, #0] // B
	ldp		q30, q31, [x10, #32] // B
	add		x10, x10, x11

	fmla	v0.2d, v20.2d, v24.d[0]
	fmla	v1.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v20.2d, v24.d[1]
	fmla	v3.2d, v21.2d, v24.d[1]
	fmla	v4.2d, v20.2d, v25.d[0]
	fmla	v5.2d, v21.2d, v25.d[0]
	fmla	v6.2d, v20.2d, v25.d[1]
	fmla	v7.2d, v21.2d, v25.d[1]
	fmla	v8.2d, v20.2d, v26.d[0]
	fmla	v9.2d, v21.2d, v26.d[0]
	fmla	v10.2d, v20.2d, v26.d[1]
	fmla	v11.2d, v21.2d, v26.d[1]
	fmla	v12.2d, v20.2d, v27.d[0]
	fmla	v13.2d, v21.2d, v27.d[0]
	fmla	v14.2d, v20.2d, v27.d[1]
	fmla	v15.2d, v21.2d, v27.d[1]

	// unroll 3
	fmla	v0.2d, v22.2d, v28.d[0]
	fmla	v1.2d, v23.2d, v28.d[0]
	fmla	v2.2d, v22.2d, v28.d[1]
	fmla	v3.2d, v23.2d, v28.d[1]
	fmla	v4.2d, v22.2d, v29.d[0]
	fmla	v5.2d, v23.2d, v29.d[0]
	fmla	v6.2d, v22.2d, v29.d[1]
	fmla	v7.2d, v23.2d, v29.d[1]
	fmla	v8.2d, v22.2d, v30.d[0]
	fmla	v9.2d, v23.2d, v30.d[0]
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
	fmla	v12.2d, v22.2d, v31.d[0]
	fmla	v13.2d, v23.2d, v31.d[0]
	fmla	v14.2d, v22.2d, v31.d[1]
	fmla	v15.2d, v23.2d, v31.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	add		x10, x10, x11
	ldp		q28, q29, [x10, #0] // B
	ldp		q30, q31, [x10, #32] // B
	add		x10, x10, x11

	ldp		q16, q17, [x9, #(0*8+0*32)]
	ldp		q18, q19, [x9, #(0*8+1*32)]
	ldp		q20, q21, [x9, #(0*8+2*32)]
	ldp		q22, q23, [x9, #(0*8+3*32)]
	add		x9, x9, #128

	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	fmla	v8.2d, v16.2d, v26.d[0]
	fmla	v9.2d, v17.2d, v26.d[0]
	fmla	v10.2d, v16.2d, v26.d[1]
	fmla	v11.2d, v17.2d, v26.d[1]
	fmla	v12.2d, v16.2d, v27.d[0]
	fmla	v13.2d, v17.2d, v27.d[0]
	fmla	v14.2d, v16.2d, v27.d[1]
	fmla	v15.2d, v17.2d, v27.d[1]

	// unroll 1
	fmla	v0.2d, v18.2d, v28.d[0]
	fmla	v1.2d, v19.2d, v28.d[0]
	fmla	v2.2d, v18.2d, v28.d[1]
	fmla	v3.2d, v19.2d, v28.d[1]
	fmla	v4.2d, v18.2d, v29.d[0]
	fmla	v5.2d, v19.2d, v29.d[0]
	fmla	v6.2d, v18.2d, v29.d[1]
	fmla	v7.2d, v19.2d, v29.d[1]
	fmla	v8.2d, v18.2d, v30.d[0]
	fmla	v9.2d, v19.2d, v30.d[0]
	fmla	v10.2d, v18.2d, v30.d[1]
	fmla	v11.2d, v19.2d, v30.d[1]
	fmla	v12.2d, v18.2d, v31.d[0]
	fmla	v13.2d, v19.2d, v31.d[0]
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]

	// unroll 2
	ldp		q24, q25, [x10, #0] // B
	ldp		q26, q27, [x10, #32] // B
	add		x10, x10, x11
	ldp		q28, q29, [x10, #0] // B
	ldp		q30, q31, [x10, #32] // B
	add		x10, x10, x11

	fmla	v0.2d, v20.2d, v24.d[0]
	fmla	v1.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v20.2d, v24.d[1]
	fmla	v3.2d, v21.2d, v24.d[1]
	fmla	v4.2d, v20.2d, v25.d[0]
	fmla	v5.2d, v21.2d, v25.d[0]
	fmla	v6.2d, v20.2d, v25.d[1]
	fmla	v7.2d, v21.2d, v25.d[1]
	fmla	v8.2d, v20.2d, v26.d[0]
	fmla	v9.2d, v21.2d, v26.d[0]
	fmla	v10.2d, v20.2d, v26.d[1]
	fmla	v11.2d, v21.2d, v26.d[1]
	fmla	v12.2d, v20.2d, v27.d[0]
	fmla	v13.2d, v21.2d, v27.d[0]
	fmla	v14.2d, v20.2d, v27.d[1]
	fmla	v15.2d, v21.2d, v27.d[1]

	// unroll 3
	fmla	v0.2d, v22.2d, v28.d[0]
	fmla	v1.2d, v23.2d, v28.d[0]
	fmla	v2.2d, v22.2d, v28.d[1]
	fmla	v3.2d, v23.2d, v28.d[1]
	fmla	v4.2d, v22.2d, v29.d[0]
	fmla	v5.2d, v23.2d, v29.d[0]
	fmla	v6.2d, v22.2d, v29.d[1]
	fmla	v7.2d, v23.2d, v29.d[1]
	fmla	v8.2d, v22.2d, v30.d[0]
	fmla	v9.2d, v23.2d, v30.d[0]
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
	fmla	v12.2d, v22.2d, v31.d[0]
	fmla	v13.2d, v23.2d, v31.d[0]
	fmla	v14.2d, v22.2d, v31.d[1]
	fmla	v15.2d, v23.2d, v31.d[1]

	sub		w8, w8, #4
	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x9, x9, #32
//	sub		x11, x11, #32
//	sub		x12, x12, #32
//	sub		x10, x10, x11

3: // clean1-up loop

	// unroll 0
	ld1		{v16.2d, v17.2d}, [x9], #32
	ldp		q24, q25, [x10]
	ldp		q26, q27, [x10, #32]
	add		x10, x10, x11

	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	fmla	v8.2d, v16.2d, v26.d[0]
	fmla	v9.2d, v17.2d, v26.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v16.2d, v26.d[1]
	fmla	v11.2d, v17.2d, v26.d[1]
	fmla	v12.2d, v16.2d, v27.d[0]
	fmla	v13.2d, v17.2d, v27.d[0]
	cmp		w8, #0
	fmla	v14.2d, v16.2d, v27.d[1]
	fmla	v15.2d, v17.2d, v27.d[1]

	bgt		3b

2: // return



#else



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x11, x11
	add		x13, x12, x11

//	add		x14, x12, #32

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x10, #0] // 0
	prfm	PLDL1KEEP, [x10, x11] // 1
	prfm	PLDL1KEEP, [x10, x12] // 2
	prfm	PLDL1KEEP, [x10, x13] // 3

	// preload
	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	add		x10, x10, x11

//	ldp		q28, q29, [x10, #(0*8+1*32)]
//	ldp		q30, q31, [x12, #(0*8+1*32)]

	ldp		q16, q17, [x9, #(0*8+0*32)]
	ldp		q18, q19, [x9, #(0*8+1*32)]
	ldp		q20, q21, [x9, #(0*8+2*32)]
	ldp		q22, q23, [x9, #(0*8+3*32)]

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// main loop
1:
	
	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q28, q29, [x10, #0] // B
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	ldp		q30, q31, [x10, #32] // B
	fmla	v4.2d, v16.2d, v25.d[0]
	add		x10, x10, x11
	fmla	v5.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x10, x12]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x10, x14]
	fmla	v8.2d, v16.2d, v26.d[0]
	fmla	v9.2d, v17.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, #(0+1*128)]
	fmla	v10.2d, v16.2d, v26.d[1]
	fmla	v11.2d, v17.2d, v26.d[1]
	prfm	PLDL1KEEP, [x9, #(64+1*128)]
	fmla	v12.2d, v16.2d, v27.d[0]
	fmla	v13.2d, v17.2d, v27.d[0]
	fmla	v14.2d, v16.2d, v27.d[1]
	fmla	v15.2d, v17.2d, v27.d[1]

	// unroll 1
	fmla	v0.2d, v18.2d, v28.d[0]
	fmla	v1.2d, v19.2d, v28.d[0]
	ldp		q24, q25, [x10, #0] // B
	fmla	v2.2d, v18.2d, v28.d[1]
	fmla	v3.2d, v19.2d, v28.d[1]
	ldp		q26, q27, [x10, #32] // B
	fmla	v4.2d, v18.2d, v29.d[0]
	fmla	v5.2d, v19.2d, v29.d[0]
	add		x10, x10, x11
	fmla	v6.2d, v18.2d, v29.d[1]
	fmla	v7.2d, v19.2d, v29.d[1]
	prfm	PLDL1KEEP, [x10, x12]
	fmla	v8.2d, v18.2d, v30.d[0]
	fmla	v9.2d, v19.2d, v30.d[0]
//	prfm	PLDL1KEEP, [x10, x14]
	fmla	v10.2d, v18.2d, v30.d[1]
	fmla	v11.2d, v19.2d, v30.d[1]
	add		x9, x9, #128
	fmla	v12.2d, v18.2d, v31.d[0]
	fmla	v13.2d, v19.2d, v31.d[0]
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]

	// unroll 2
	fmla	v0.2d, v20.2d, v24.d[0]
	fmla	v1.2d, v21.2d, v24.d[0]
	ldp		q28, q29, [x10, #0] // B
	fmla	v2.2d, v20.2d, v24.d[1]
	fmla	v3.2d, v21.2d, v24.d[1]
	ldp		q30, q31, [x10, #32] // B
	fmla	v4.2d, v20.2d, v25.d[0]
	fmla	v5.2d, v21.2d, v25.d[0]
	add		x10, x10, x11
	fmla	v6.2d, v20.2d, v25.d[1]
	fmla	v7.2d, v21.2d, v25.d[1]
	prfm	PLDL1KEEP, [x10, x12]
	fmla	v8.2d, v20.2d, v26.d[0]
	fmla	v9.2d, v21.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x10, x14]
	fmla	v10.2d, v20.2d, v26.d[1]
	fmla	v11.2d, v21.2d, v26.d[1]
	sub		w8, w8, #4
	fmla	v12.2d, v20.2d, v27.d[0]
	fmla	v13.2d, v21.2d, v27.d[0]
	ldp		q24, q25, [x10, #0] // B
	fmla	v14.2d, v20.2d, v27.d[1]
	fmla	v15.2d, v21.2d, v27.d[1]
	ldp		q26, q27, [x10, #32] // B

	// unroll 3
	fmla	v0.2d, v22.2d, v28.d[0]
	fmla	v1.2d, v23.2d, v28.d[0]
	add		x10, x10, x11
	fmla	v2.2d, v22.2d, v28.d[1]
	fmla	v3.2d, v23.2d, v28.d[1]
	ldp		q16, q17, [x9, #(0*8+0*32)] // A
	fmla	v4.2d, v22.2d, v29.d[0]
	fmla	v5.2d, v23.2d, v29.d[0]
	prfm	PLDL1KEEP, [x10, x12]
	fmla	v6.2d, v22.2d, v29.d[1]
	fmla	v7.2d, v23.2d, v29.d[1]
	ldp		q18, q19, [x9, #(0*8+1*32)] // A
	fmla	v8.2d, v22.2d, v30.d[0]
	fmla	v9.2d, v23.2d, v30.d[0]
//	prfm	PLDL1KEEP, [x10, x14]
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
	ldp		q20, q21, [x9, #(0*8+2*32)] // A
	fmla	v12.2d, v22.2d, v31.d[0]
	fmla	v13.2d, v23.2d, v31.d[0]
	cmp		w8, #4
	fmla	v14.2d, v22.2d, v31.d[1]
	fmla	v15.2d, v23.2d, v31.d[1]
	ldp		q22, q23, [x9, #(0*8+3*32)] // A


	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q28, q29, [x10, #0] // B
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	ldp		q30, q31, [x10, #32] // B
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	add		x10, x10, x11
//	prfm	PLDL1KEEP, [x10, #(0+1*128)]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x12, #(0+1*128)]
	fmla	v8.2d, v16.2d, v26.d[0]
	fmla	v9.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x10, #(64+1*128)]
	fmla	v10.2d, v16.2d, v26.d[1]
	fmla	v11.2d, v17.2d, v26.d[1]
//	prfm	PLDL1KEEP, [x12, #(64+1*128)]
	fmla	v12.2d, v16.2d, v27.d[0]
	fmla	v13.2d, v17.2d, v27.d[0]
//	prfm	PLDL1KEEP, [x9, #(0+1*128)]
	fmla	v14.2d, v16.2d, v27.d[1]
	fmla	v15.2d, v17.2d, v27.d[1]
//	prfm	PLDL1KEEP, [x9, #(64+1*128)]

	// unroll 1
	fmla	v0.2d, v18.2d, v28.d[0]
	fmla	v1.2d, v19.2d, v28.d[0]
	ldp		q24, q25, [x10, #0] // B
	fmla	v2.2d, v18.2d, v28.d[1]
	fmla	v3.2d, v19.2d, v28.d[1]
	ldp		q26, q27, [x10, #32] // B
	fmla	v4.2d, v18.2d, v29.d[0]
	fmla	v5.2d, v19.2d, v29.d[0]
	add		x10, x10, x11
	fmla	v6.2d, v18.2d, v29.d[1]
	fmla	v7.2d, v19.2d, v29.d[1]
	fmla	v8.2d, v18.2d, v30.d[0]
	fmla	v9.2d, v19.2d, v30.d[0]
	add		x9, x9, #128
	fmla	v10.2d, v18.2d, v30.d[1]
	fmla	v11.2d, v19.2d, v30.d[1]
	fmla	v12.2d, v18.2d, v31.d[0]
	fmla	v13.2d, v19.2d, v31.d[0]
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]

	// unroll 2
	fmla	v0.2d, v20.2d, v24.d[0]
	fmla	v1.2d, v21.2d, v24.d[0]
	ldp		q28, q29, [x10, #0] // B
	fmla	v2.2d, v20.2d, v24.d[1]
	fmla	v3.2d, v21.2d, v24.d[1]
	ldp		q30, q31, [x10, #32] // B
	fmla	v4.2d, v20.2d, v25.d[0]
	fmla	v5.2d, v21.2d, v25.d[0]
	add		x10, x10, x11
	fmla	v6.2d, v20.2d, v25.d[1]
	fmla	v7.2d, v21.2d, v25.d[1]
	add		x12, x12, #128
	fmla	v8.2d, v20.2d, v26.d[0]
	fmla	v9.2d, v21.2d, v26.d[0]
	fmla	v10.2d, v20.2d, v26.d[1]
	fmla	v11.2d, v21.2d, v26.d[1]
	sub		w8, w8, #4
	fmla	v12.2d, v20.2d, v27.d[0]
	fmla	v13.2d, v21.2d, v27.d[0]
//	ldp		q24, q25, [x10, #0] // B
	fmla	v14.2d, v20.2d, v27.d[1]
	fmla	v15.2d, v21.2d, v27.d[1]
//	ldp		q26, q27, [x12, #32] // B

	// unroll 3
	fmla	v0.2d, v22.2d, v28.d[0]
	fmla	v1.2d, v23.2d, v28.d[0]
//	add		x10, x10, x11
	cmp		w8, #4
	fmla	v2.2d, v22.2d, v28.d[1]
	fmla	v3.2d, v23.2d, v28.d[1]
//	ldp		q16, q17, [x9, #(0*8+0*32)] // A
	fmla	v4.2d, v22.2d, v29.d[0]
	fmla	v5.2d, v23.2d, v29.d[0]
	fmla	v6.2d, v22.2d, v29.d[1]
	fmla	v7.2d, v23.2d, v29.d[1]
//	ldp		q18, q19, [x9, #(0*8+1*32)] // A
	fmla	v8.2d, v22.2d, v30.d[0]
	fmla	v9.2d, v23.2d, v30.d[0]
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
//	ldp		q20, q21, [x9, #(0*8+2*32)] // A
	fmla	v12.2d, v22.2d, v31.d[0]
	fmla	v13.2d, v23.2d, v31.d[0]
	fmla	v14.2d, v22.2d, v31.d[1]
	fmla	v15.2d, v23.2d, v31.d[1]
//	ldp		q22, q23, [x9, #(0*8+3*32)] // A

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x9, x9, #32
//	sub		x11, x11, #32
//	sub		x12, x12, #32
	sub		x10, x10, x11

3: // clean1-up loop

	// unroll 0
	ld1		{v16.2d, v17.2d}, [x9], #32
	ldp		q24, q25, [x10]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	ldp		q26, q27, [x10, #32]
	fmla	v8.2d, v16.2d, v26.d[0]
	fmla	v9.2d, v17.2d, v26.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v16.2d, v26.d[1]
	fmla	v11.2d, v17.2d, v26.d[1]
	add		x10, x10, x11
	fmla	v12.2d, v16.2d, v27.d[0]
	fmla	v13.2d, v17.2d, v27.d[0]
	cmp		w8, #0
	fmla	v14.2d, v16.2d, v27.d[1]
	fmla	v15.2d, v17.2d, v27.d[1]

	bgt		3b

2: // return



#endif



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_4x8_lib4c)
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
	.macro INNER_KERNEL_GEMM_ADD_NT_4X7_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_4x7_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x11, x11
	add		x13, x12, x11

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// main loop
1:
	
	// unroll 0
	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	add		x10, x10, x11
	ldp		q28, q29, [x10, #0] // B
	ldp		q30, q31, [x10, #32] // B
	add		x10, x10, x11

	ldp		q16, q17, [x9, #(0*8+0*32)]
	ldp		q18, q19, [x9, #(0*8+1*32)]
	ldp		q20, q21, [x9, #(0*8+2*32)]
	ldp		q22, q23, [x9, #(0*8+3*32)]
	add		x9, x9, #128

	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	fmla	v8.2d, v16.2d, v26.d[0]
	fmla	v9.2d, v17.2d, v26.d[0]
	fmla	v10.2d, v16.2d, v26.d[1]
	fmla	v11.2d, v17.2d, v26.d[1]
	fmla	v12.2d, v16.2d, v27.d[0]
	fmla	v13.2d, v17.2d, v27.d[0]
//	fmla	v14.2d, v16.2d, v27.d[1]
//	fmla	v15.2d, v17.2d, v27.d[1]

	// unroll 1
	fmla	v0.2d, v18.2d, v28.d[0]
	fmla	v1.2d, v19.2d, v28.d[0]
	fmla	v2.2d, v18.2d, v28.d[1]
	fmla	v3.2d, v19.2d, v28.d[1]
	fmla	v4.2d, v18.2d, v29.d[0]
	fmla	v5.2d, v19.2d, v29.d[0]
	fmla	v6.2d, v18.2d, v29.d[1]
	fmla	v7.2d, v19.2d, v29.d[1]
	fmla	v8.2d, v18.2d, v30.d[0]
	fmla	v9.2d, v19.2d, v30.d[0]
	fmla	v10.2d, v18.2d, v30.d[1]
	fmla	v11.2d, v19.2d, v30.d[1]
	fmla	v12.2d, v18.2d, v31.d[0]
	fmla	v13.2d, v19.2d, v31.d[0]
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]

	// unroll 2
	ldp		q24, q25, [x10, #0] // B
	ldp		q26, q27, [x10, #32] // B
	add		x10, x10, x11
	ldp		q28, q29, [x10, #0] // B
	ldp		q30, q31, [x10, #32] // B
	add		x10, x10, x11

	fmla	v0.2d, v20.2d, v24.d[0]
	fmla	v1.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v20.2d, v24.d[1]
	fmla	v3.2d, v21.2d, v24.d[1]
	fmla	v4.2d, v20.2d, v25.d[0]
	fmla	v5.2d, v21.2d, v25.d[0]
	fmla	v6.2d, v20.2d, v25.d[1]
	fmla	v7.2d, v21.2d, v25.d[1]
	fmla	v8.2d, v20.2d, v26.d[0]
	fmla	v9.2d, v21.2d, v26.d[0]
	fmla	v10.2d, v20.2d, v26.d[1]
	fmla	v11.2d, v21.2d, v26.d[1]
	fmla	v12.2d, v20.2d, v27.d[0]
	fmla	v13.2d, v21.2d, v27.d[0]
//	fmla	v14.2d, v20.2d, v27.d[1]
//	fmla	v15.2d, v21.2d, v27.d[1]

	// unroll 3
	fmla	v0.2d, v22.2d, v28.d[0]
	fmla	v1.2d, v23.2d, v28.d[0]
	fmla	v2.2d, v22.2d, v28.d[1]
	fmla	v3.2d, v23.2d, v28.d[1]
	fmla	v4.2d, v22.2d, v29.d[0]
	fmla	v5.2d, v23.2d, v29.d[0]
	fmla	v6.2d, v22.2d, v29.d[1]
	fmla	v7.2d, v23.2d, v29.d[1]
	fmla	v8.2d, v22.2d, v30.d[0]
	fmla	v9.2d, v23.2d, v30.d[0]
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
	fmla	v12.2d, v22.2d, v31.d[0]
	fmla	v13.2d, v23.2d, v31.d[0]
//	fmla	v14.2d, v22.2d, v31.d[1]
//	fmla	v15.2d, v23.2d, v31.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	add		x10, x10, x11
	ldp		q28, q29, [x10, #0] // B
	ldp		q30, q31, [x10, #32] // B
	add		x10, x10, x11

	ldp		q16, q17, [x9, #(0*8+0*32)]
	ldp		q18, q19, [x9, #(0*8+1*32)]
	ldp		q20, q21, [x9, #(0*8+2*32)]
	ldp		q22, q23, [x9, #(0*8+3*32)]
	add		x9, x9, #128

	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	fmla	v8.2d, v16.2d, v26.d[0]
	fmla	v9.2d, v17.2d, v26.d[0]
	fmla	v10.2d, v16.2d, v26.d[1]
	fmla	v11.2d, v17.2d, v26.d[1]
	fmla	v12.2d, v16.2d, v27.d[0]
	fmla	v13.2d, v17.2d, v27.d[0]
//	fmla	v14.2d, v16.2d, v27.d[1]
//	fmla	v15.2d, v17.2d, v27.d[1]

	// unroll 1
	fmla	v0.2d, v18.2d, v28.d[0]
	fmla	v1.2d, v19.2d, v28.d[0]
	fmla	v2.2d, v18.2d, v28.d[1]
	fmla	v3.2d, v19.2d, v28.d[1]
	fmla	v4.2d, v18.2d, v29.d[0]
	fmla	v5.2d, v19.2d, v29.d[0]
	fmla	v6.2d, v18.2d, v29.d[1]
	fmla	v7.2d, v19.2d, v29.d[1]
	fmla	v8.2d, v18.2d, v30.d[0]
	fmla	v9.2d, v19.2d, v30.d[0]
	fmla	v10.2d, v18.2d, v30.d[1]
	fmla	v11.2d, v19.2d, v30.d[1]
	fmla	v12.2d, v18.2d, v31.d[0]
	fmla	v13.2d, v19.2d, v31.d[0]
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]

	// unroll 2
	ldp		q24, q25, [x10, #0] // B
	ldp		q26, q27, [x10, #32] // B
	add		x10, x10, x11
	ldp		q28, q29, [x10, #0] // B
	ldp		q30, q31, [x10, #32] // B
	add		x10, x10, x11

	fmla	v0.2d, v20.2d, v24.d[0]
	fmla	v1.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v20.2d, v24.d[1]
	fmla	v3.2d, v21.2d, v24.d[1]
	fmla	v4.2d, v20.2d, v25.d[0]
	fmla	v5.2d, v21.2d, v25.d[0]
	fmla	v6.2d, v20.2d, v25.d[1]
	fmla	v7.2d, v21.2d, v25.d[1]
	fmla	v8.2d, v20.2d, v26.d[0]
	fmla	v9.2d, v21.2d, v26.d[0]
	fmla	v10.2d, v20.2d, v26.d[1]
	fmla	v11.2d, v21.2d, v26.d[1]
	fmla	v12.2d, v20.2d, v27.d[0]
	fmla	v13.2d, v21.2d, v27.d[0]
//	fmla	v14.2d, v20.2d, v27.d[1]
//	fmla	v15.2d, v21.2d, v27.d[1]

	// unroll 3
	fmla	v0.2d, v22.2d, v28.d[0]
	fmla	v1.2d, v23.2d, v28.d[0]
	fmla	v2.2d, v22.2d, v28.d[1]
	fmla	v3.2d, v23.2d, v28.d[1]
	fmla	v4.2d, v22.2d, v29.d[0]
	fmla	v5.2d, v23.2d, v29.d[0]
	fmla	v6.2d, v22.2d, v29.d[1]
	fmla	v7.2d, v23.2d, v29.d[1]
	fmla	v8.2d, v22.2d, v30.d[0]
	fmla	v9.2d, v23.2d, v30.d[0]
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
	fmla	v12.2d, v22.2d, v31.d[0]
	fmla	v13.2d, v23.2d, v31.d[0]
//	fmla	v14.2d, v22.2d, v31.d[1]
//	fmla	v15.2d, v23.2d, v31.d[1]

	sub		w8, w8, #4
	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x9, x9, #32
//	sub		x11, x11, #32
//	sub		x12, x12, #32
//	sub		x10, x10, x11

3: // clean1-up loop

	// unroll 0
	ld1		{v16.2d, v17.2d}, [x9], #32
	ldp		q24, q25, [x10]
	ldp		q26, q27, [x10, #32]
	add		x10, x10, x11

	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	fmla	v8.2d, v16.2d, v26.d[0]
	fmla	v9.2d, v17.2d, v26.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v16.2d, v26.d[1]
	fmla	v11.2d, v17.2d, v26.d[1]
	fmla	v12.2d, v16.2d, v27.d[0]
	fmla	v13.2d, v17.2d, v27.d[0]
	cmp		w8, #0
//	fmla	v14.2d, v16.2d, v27.d[1]
//	fmla	v15.2d, v17.2d, v27.d[1]

	bgt		3b

2: // return



#else



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x11, x11
	add		x13, x12, x11

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x10, #0] // 0
	prfm	PLDL1KEEP, [x10, x11] // 1
	prfm	PLDL1KEEP, [x10, x12] // 2
	prfm	PLDL1KEEP, [x10, x13] // 3

	// preload
	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	add		x10, x10, x11

//	ldp		q28, q29, [x10, #(0*8+1*32)]
//	ldp		q30, q31, [x12, #(0*8+1*32)]

	ldp		q16, q17, [x9, #(0*8+0*32)]
	ldp		q18, q19, [x9, #(0*8+1*32)]
	ldp		q20, q21, [x9, #(0*8+2*32)]
	ldp		q22, q23, [x9, #(0*8+3*32)]

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// main loop
1:
	
	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q28, q29, [x10, #0] // B
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	ldp		q30, q31, [x10, #32] // B
	fmla	v4.2d, v16.2d, v25.d[0]
	add		x10, x10, x11
	fmla	v5.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x10, x12]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x12, #(0+1*128)]
	fmla	v8.2d, v16.2d, v26.d[0]
	fmla	v9.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x10, #(64+1*128)]
	fmla	v10.2d, v16.2d, v26.d[1]
	fmla	v11.2d, v17.2d, v26.d[1]
//	prfm	PLDL1KEEP, [x12, #(64+1*128)]
	fmla	v12.2d, v16.2d, v27.d[0]
	fmla	v13.2d, v17.2d, v27.d[0]
	prfm	PLDL1KEEP, [x9, #(0+1*128)]
//	fmla	v14.2d, v16.2d, v27.d[1]
//	fmla	v15.2d, v17.2d, v27.d[1]
	prfm	PLDL1KEEP, [x9, #(64+1*128)]

	// unroll 1
	fmla	v0.2d, v18.2d, v28.d[0]
	fmla	v1.2d, v19.2d, v28.d[0]
	ldp		q24, q25, [x10, #0] // B
	fmla	v2.2d, v18.2d, v28.d[1]
	fmla	v3.2d, v19.2d, v28.d[1]
	ldp		q26, q27, [x10, #32] // B
	fmla	v4.2d, v18.2d, v29.d[0]
	fmla	v5.2d, v19.2d, v29.d[0]
	add		x10, x10, x11
	fmla	v6.2d, v18.2d, v29.d[1]
	fmla	v7.2d, v19.2d, v29.d[1]
	prfm	PLDL1KEEP, [x10, x12]
	fmla	v8.2d, v18.2d, v30.d[0]
	fmla	v9.2d, v19.2d, v30.d[0]
	add		x9, x9, #128
	fmla	v10.2d, v18.2d, v30.d[1]
	fmla	v11.2d, v19.2d, v30.d[1]
	fmla	v12.2d, v18.2d, v31.d[0]
	fmla	v13.2d, v19.2d, v31.d[0]
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]

	// unroll 2
	fmla	v0.2d, v20.2d, v24.d[0]
	fmla	v1.2d, v21.2d, v24.d[0]
	ldp		q28, q29, [x10, #0] // B
	fmla	v2.2d, v20.2d, v24.d[1]
	fmla	v3.2d, v21.2d, v24.d[1]
	ldp		q30, q31, [x10, #32] // B
	fmla	v4.2d, v20.2d, v25.d[0]
	fmla	v5.2d, v21.2d, v25.d[0]
	add		x10, x10, x11
	fmla	v6.2d, v20.2d, v25.d[1]
	fmla	v7.2d, v21.2d, v25.d[1]
	prfm	PLDL1KEEP, [x10, x12]
	fmla	v8.2d, v20.2d, v26.d[0]
	fmla	v9.2d, v21.2d, v26.d[0]
	fmla	v10.2d, v20.2d, v26.d[1]
	fmla	v11.2d, v21.2d, v26.d[1]
	sub		w8, w8, #4
	fmla	v12.2d, v20.2d, v27.d[0]
	fmla	v13.2d, v21.2d, v27.d[0]
	ldp		q24, q25, [x10, #0] // B
//	fmla	v14.2d, v20.2d, v27.d[1]
//	fmla	v15.2d, v21.2d, v27.d[1]
	ldp		q26, q27, [x10, #32] // B

	// unroll 3
	fmla	v0.2d, v22.2d, v28.d[0]
	fmla	v1.2d, v23.2d, v28.d[0]
	add		x10, x10, x11
	fmla	v2.2d, v22.2d, v28.d[1]
	fmla	v3.2d, v23.2d, v28.d[1]
	ldp		q16, q17, [x9, #(0*8+0*32)] // A
	fmla	v4.2d, v22.2d, v29.d[0]
	fmla	v5.2d, v23.2d, v29.d[0]
	prfm	PLDL1KEEP, [x10, x12]
	fmla	v6.2d, v22.2d, v29.d[1]
	fmla	v7.2d, v23.2d, v29.d[1]
	ldp		q18, q19, [x9, #(0*8+1*32)] // A
	fmla	v8.2d, v22.2d, v30.d[0]
	fmla	v9.2d, v23.2d, v30.d[0]
	cmp		w8, #4
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
	ldp		q20, q21, [x9, #(0*8+2*32)] // A
	fmla	v12.2d, v22.2d, v31.d[0]
	fmla	v13.2d, v23.2d, v31.d[0]
//	fmla	v14.2d, v22.2d, v31.d[1]
//	fmla	v15.2d, v23.2d, v31.d[1]
	ldp		q22, q23, [x9, #(0*8+3*32)] // A


	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q28, q29, [x10, #0] // B
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	ldp		q30, q31, [x10, #32] // B
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	add		x10, x10, x11
//	prfm	PLDL1KEEP, [x10, #(0+1*128)]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x12, #(0+1*128)]
	fmla	v8.2d, v16.2d, v26.d[0]
	fmla	v9.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x10, #(64+1*128)]
	fmla	v10.2d, v16.2d, v26.d[1]
	fmla	v11.2d, v17.2d, v26.d[1]
//	prfm	PLDL1KEEP, [x12, #(64+1*128)]
	fmla	v12.2d, v16.2d, v27.d[0]
	fmla	v13.2d, v17.2d, v27.d[0]
//	prfm	PLDL1KEEP, [x9, #(0+1*128)]
//	fmla	v14.2d, v16.2d, v27.d[1]
//	fmla	v15.2d, v17.2d, v27.d[1]
//	prfm	PLDL1KEEP, [x9, #(64+1*128)]

	// unroll 1
	fmla	v0.2d, v18.2d, v28.d[0]
	fmla	v1.2d, v19.2d, v28.d[0]
	ldp		q24, q25, [x10, #0] // B
	fmla	v2.2d, v18.2d, v28.d[1]
	fmla	v3.2d, v19.2d, v28.d[1]
	ldp		q26, q27, [x10, #32] // B
	fmla	v4.2d, v18.2d, v29.d[0]
	fmla	v5.2d, v19.2d, v29.d[0]
	add		x10, x10, x11
	fmla	v6.2d, v18.2d, v29.d[1]
	fmla	v7.2d, v19.2d, v29.d[1]
	fmla	v8.2d, v18.2d, v30.d[0]
	fmla	v9.2d, v19.2d, v30.d[0]
	add		x9, x9, #128
	fmla	v10.2d, v18.2d, v30.d[1]
	fmla	v11.2d, v19.2d, v30.d[1]
	fmla	v12.2d, v18.2d, v31.d[0]
	fmla	v13.2d, v19.2d, v31.d[0]
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]

	// unroll 2
	fmla	v0.2d, v20.2d, v24.d[0]
	fmla	v1.2d, v21.2d, v24.d[0]
	ldp		q28, q29, [x10, #0] // B
	fmla	v2.2d, v20.2d, v24.d[1]
	fmla	v3.2d, v21.2d, v24.d[1]
	ldp		q30, q31, [x10, #32] // B
	fmla	v4.2d, v20.2d, v25.d[0]
	fmla	v5.2d, v21.2d, v25.d[0]
	add		x10, x10, x11
	fmla	v6.2d, v20.2d, v25.d[1]
	fmla	v7.2d, v21.2d, v25.d[1]
	add		x12, x12, #128
	fmla	v8.2d, v20.2d, v26.d[0]
	fmla	v9.2d, v21.2d, v26.d[0]
	fmla	v10.2d, v20.2d, v26.d[1]
	fmla	v11.2d, v21.2d, v26.d[1]
	sub		w8, w8, #4
	fmla	v12.2d, v20.2d, v27.d[0]
	fmla	v13.2d, v21.2d, v27.d[0]
//	ldp		q24, q25, [x10, #0] // B
//	fmla	v14.2d, v20.2d, v27.d[1]
//	fmla	v15.2d, v21.2d, v27.d[1]
//	ldp		q26, q27, [x12, #32] // B

	// unroll 3
	fmla	v0.2d, v22.2d, v28.d[0]
	fmla	v1.2d, v23.2d, v28.d[0]
//	add		x10, x10, x11
	cmp		w8, #4
	fmla	v2.2d, v22.2d, v28.d[1]
	fmla	v3.2d, v23.2d, v28.d[1]
//	ldp		q16, q17, [x9, #(0*8+0*32)] // A
	fmla	v4.2d, v22.2d, v29.d[0]
	fmla	v5.2d, v23.2d, v29.d[0]
	fmla	v6.2d, v22.2d, v29.d[1]
	fmla	v7.2d, v23.2d, v29.d[1]
//	ldp		q18, q19, [x9, #(0*8+1*32)] // A
	fmla	v8.2d, v22.2d, v30.d[0]
	fmla	v9.2d, v23.2d, v30.d[0]
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
//	ldp		q20, q21, [x9, #(0*8+2*32)] // A
	fmla	v12.2d, v22.2d, v31.d[0]
	fmla	v13.2d, v23.2d, v31.d[0]
//	fmla	v14.2d, v22.2d, v31.d[1]
//	fmla	v15.2d, v23.2d, v31.d[1]
//	ldp		q22, q23, [x9, #(0*8+3*32)] // A

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x9, x9, #32
//	sub		x11, x11, #32
//	sub		x12, x12, #32
	sub		x10, x10, x11

3: // clean1-up loop

	// unroll 0
	ld1		{v16.2d, v17.2d}, [x9], #32
	ldp		q24, q25, [x10]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	ldp		q26, q27, [x10, #32]
	fmla	v8.2d, v16.2d, v26.d[0]
	fmla	v9.2d, v17.2d, v26.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v16.2d, v26.d[1]
	fmla	v11.2d, v17.2d, v26.d[1]
	add		x10, x10, x11
	fmla	v12.2d, v16.2d, v27.d[0]
	fmla	v13.2d, v17.2d, v27.d[0]
	cmp		w8, #0
//	fmla	v14.2d, v16.2d, v27.d[1]
//	fmla	v15.2d, v17.2d, v27.d[1]

	bgt		3b

2: // return



#endif



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_4x7_lib4c)
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
	.macro INNER_KERNEL_GEMM_ADD_NT_4X6_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_4x6_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x11, x11
	add		x13, x12, x11

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// main loop
1:
	
	// unroll 0
	ldp		q24, q25, [x10, #0]
//	ldp		q26, q27, [x10, #32]
	ldr		q26, [x10, #32]
	add		x10, x10, x11
	ldp		q28, q29, [x10, #0] // B
//	ldp		q30, q31, [x10, #32] // B
	ldr		q30, [x10, #32] // B
	add		x10, x10, x11

	ldp		q16, q17, [x9, #(0*8+0*32)]
	ldp		q18, q19, [x9, #(0*8+1*32)]
	ldp		q20, q21, [x9, #(0*8+2*32)]
	ldp		q22, q23, [x9, #(0*8+3*32)]
	add		x9, x9, #128

	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	fmla	v8.2d, v16.2d, v26.d[0]
	fmla	v9.2d, v17.2d, v26.d[0]
	fmla	v10.2d, v16.2d, v26.d[1]
	fmla	v11.2d, v17.2d, v26.d[1]
//	fmla	v12.2d, v16.2d, v27.d[0]
//	fmla	v13.2d, v17.2d, v27.d[0]
//	fmla	v14.2d, v16.2d, v27.d[1]
//	fmla	v15.2d, v17.2d, v27.d[1]

	// unroll 1
	fmla	v0.2d, v18.2d, v28.d[0]
	fmla	v1.2d, v19.2d, v28.d[0]
	fmla	v2.2d, v18.2d, v28.d[1]
	fmla	v3.2d, v19.2d, v28.d[1]
	fmla	v4.2d, v18.2d, v29.d[0]
	fmla	v5.2d, v19.2d, v29.d[0]
	fmla	v6.2d, v18.2d, v29.d[1]
	fmla	v7.2d, v19.2d, v29.d[1]
	fmla	v8.2d, v18.2d, v30.d[0]
	fmla	v9.2d, v19.2d, v30.d[0]
	fmla	v10.2d, v18.2d, v30.d[1]
	fmla	v11.2d, v19.2d, v30.d[1]
//	fmla	v12.2d, v18.2d, v31.d[0]
//	fmla	v13.2d, v19.2d, v31.d[0]
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]

	// unroll 2
	ldp		q24, q25, [x10, #0] // B
//	ldp		q26, q27, [x10, #32] // B
	ldr		q26, [x10, #32] // B
	add		x10, x10, x11
	ldp		q28, q29, [x10, #0] // B
//	ldp		q30, q31, [x10, #32] // B
	ldr		q30, [x10, #32] // B
	add		x10, x10, x11

	fmla	v0.2d, v20.2d, v24.d[0]
	fmla	v1.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v20.2d, v24.d[1]
	fmla	v3.2d, v21.2d, v24.d[1]
	fmla	v4.2d, v20.2d, v25.d[0]
	fmla	v5.2d, v21.2d, v25.d[0]
	fmla	v6.2d, v20.2d, v25.d[1]
	fmla	v7.2d, v21.2d, v25.d[1]
	fmla	v8.2d, v20.2d, v26.d[0]
	fmla	v9.2d, v21.2d, v26.d[0]
	fmla	v10.2d, v20.2d, v26.d[1]
	fmla	v11.2d, v21.2d, v26.d[1]
//	fmla	v12.2d, v20.2d, v27.d[0]
//	fmla	v13.2d, v21.2d, v27.d[0]
//	fmla	v14.2d, v20.2d, v27.d[1]
//	fmla	v15.2d, v21.2d, v27.d[1]

	// unroll 3
	fmla	v0.2d, v22.2d, v28.d[0]
	fmla	v1.2d, v23.2d, v28.d[0]
	fmla	v2.2d, v22.2d, v28.d[1]
	fmla	v3.2d, v23.2d, v28.d[1]
	fmla	v4.2d, v22.2d, v29.d[0]
	fmla	v5.2d, v23.2d, v29.d[0]
	fmla	v6.2d, v22.2d, v29.d[1]
	fmla	v7.2d, v23.2d, v29.d[1]
	fmla	v8.2d, v22.2d, v30.d[0]
	fmla	v9.2d, v23.2d, v30.d[0]
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
//	fmla	v12.2d, v22.2d, v31.d[0]
//	fmla	v13.2d, v23.2d, v31.d[0]
//	fmla	v14.2d, v22.2d, v31.d[1]
//	fmla	v15.2d, v23.2d, v31.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	ldp		q24, q25, [x10, #0]
//	ldp		q26, q27, [x10, #32]
	ldr		q26, [x10, #32]
	add		x10, x10, x11
	ldp		q28, q29, [x10, #0] // B
//	ldp		q30, q31, [x10, #32] // B
	ldr		q30, [x10, #32] // B
	add		x10, x10, x11

	ldp		q16, q17, [x9, #(0*8+0*32)]
	ldp		q18, q19, [x9, #(0*8+1*32)]
	ldp		q20, q21, [x9, #(0*8+2*32)]
	ldp		q22, q23, [x9, #(0*8+3*32)]
	add		x9, x9, #128

	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	fmla	v8.2d, v16.2d, v26.d[0]
	fmla	v9.2d, v17.2d, v26.d[0]
	fmla	v10.2d, v16.2d, v26.d[1]
	fmla	v11.2d, v17.2d, v26.d[1]
//	fmla	v12.2d, v16.2d, v27.d[0]
//	fmla	v13.2d, v17.2d, v27.d[0]
//	fmla	v14.2d, v16.2d, v27.d[1]
//	fmla	v15.2d, v17.2d, v27.d[1]

	// unroll 1
	fmla	v0.2d, v18.2d, v28.d[0]
	fmla	v1.2d, v19.2d, v28.d[0]
	fmla	v2.2d, v18.2d, v28.d[1]
	fmla	v3.2d, v19.2d, v28.d[1]
	fmla	v4.2d, v18.2d, v29.d[0]
	fmla	v5.2d, v19.2d, v29.d[0]
	fmla	v6.2d, v18.2d, v29.d[1]
	fmla	v7.2d, v19.2d, v29.d[1]
	fmla	v8.2d, v18.2d, v30.d[0]
	fmla	v9.2d, v19.2d, v30.d[0]
	fmla	v10.2d, v18.2d, v30.d[1]
	fmla	v11.2d, v19.2d, v30.d[1]
//	fmla	v12.2d, v18.2d, v31.d[0]
//	fmla	v13.2d, v19.2d, v31.d[0]
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]

	// unroll 2
	ldp		q24, q25, [x10, #0] // B
//	ldp		q26, q27, [x10, #32] // B
	ldr		q26, [x10, #32] // B
	add		x10, x10, x11
	ldp		q28, q29, [x10, #0] // B
//	ldp		q30, q31, [x10, #32] // B
	ldr		q30, [x10, #32] // B
	add		x10, x10, x11

	fmla	v0.2d, v20.2d, v24.d[0]
	fmla	v1.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v20.2d, v24.d[1]
	fmla	v3.2d, v21.2d, v24.d[1]
	fmla	v4.2d, v20.2d, v25.d[0]
	fmla	v5.2d, v21.2d, v25.d[0]
	fmla	v6.2d, v20.2d, v25.d[1]
	fmla	v7.2d, v21.2d, v25.d[1]
	fmla	v8.2d, v20.2d, v26.d[0]
	fmla	v9.2d, v21.2d, v26.d[0]
	fmla	v10.2d, v20.2d, v26.d[1]
	fmla	v11.2d, v21.2d, v26.d[1]
//	fmla	v12.2d, v20.2d, v27.d[0]
//	fmla	v13.2d, v21.2d, v27.d[0]
//	fmla	v14.2d, v20.2d, v27.d[1]
//	fmla	v15.2d, v21.2d, v27.d[1]

	// unroll 3
	fmla	v0.2d, v22.2d, v28.d[0]
	fmla	v1.2d, v23.2d, v28.d[0]
	fmla	v2.2d, v22.2d, v28.d[1]
	fmla	v3.2d, v23.2d, v28.d[1]
	fmla	v4.2d, v22.2d, v29.d[0]
	fmla	v5.2d, v23.2d, v29.d[0]
	fmla	v6.2d, v22.2d, v29.d[1]
	fmla	v7.2d, v23.2d, v29.d[1]
	fmla	v8.2d, v22.2d, v30.d[0]
	fmla	v9.2d, v23.2d, v30.d[0]
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
//	fmla	v12.2d, v22.2d, v31.d[0]
//	fmla	v13.2d, v23.2d, v31.d[0]
//	fmla	v14.2d, v22.2d, v31.d[1]
//	fmla	v15.2d, v23.2d, v31.d[1]

	sub		w8, w8, #4
	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x9, x9, #32
//	sub		x11, x11, #32
//	sub		x12, x12, #32
//	sub		x10, x10, x11

3: // clean1-up loop

	// unroll 0
	ld1		{v16.2d, v17.2d}, [x9], #32
	ldp		q24, q25, [x10]
//	ldp		q26, q27, [x10, #32]
	ldr		q26, [x10, #32]
	add		x10, x10, x11

	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	fmla	v8.2d, v16.2d, v26.d[0]
	fmla	v9.2d, v17.2d, v26.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v16.2d, v26.d[1]
	fmla	v11.2d, v17.2d, v26.d[1]
//	fmla	v12.2d, v16.2d, v27.d[0]
//	fmla	v13.2d, v17.2d, v27.d[0]
	cmp		w8, #0
//	fmla	v14.2d, v16.2d, v27.d[1]
//	fmla	v15.2d, v17.2d, v27.d[1]

	bgt		3b

2: // return



#else



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x11, x11
	add		x13, x12, x11

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x10, #0] // 0
	prfm	PLDL1KEEP, [x10, x11] // 1
	prfm	PLDL1KEEP, [x10, x12] // 2
	prfm	PLDL1KEEP, [x10, x13] // 3

	// preload
	ldp		q24, q25, [x10, #0]
//	ldp		q26, q27, [x10, #32]
	ldr		q26, [x10, #32]
	add		x10, x10, x11

//	ldp		q28, q29, [x10, #(0*8+1*32)]
//	ldp		q30, q31, [x12, #(0*8+1*32)]

	ldp		q16, q17, [x9, #(0*8+0*32)]
	ldp		q18, q19, [x9, #(0*8+1*32)]
	ldp		q20, q21, [x9, #(0*8+2*32)]
	ldp		q22, q23, [x9, #(0*8+3*32)]

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// main loop
1:
	
	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q28, q29, [x10, #0] // B
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	ldp		q30, q31, [x10, #32] // B
	ldr		q30, [x10, #32] // B
	fmla	v4.2d, v16.2d, v25.d[0]
	add		x10, x10, x11
	fmla	v5.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x10, x12]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x12, #(0+1*128)]
	fmla	v8.2d, v16.2d, v26.d[0]
	fmla	v9.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x10, #(64+1*128)]
	fmla	v10.2d, v16.2d, v26.d[1]
	fmla	v11.2d, v17.2d, v26.d[1]
//	prfm	PLDL1KEEP, [x12, #(64+1*128)]
//	fmla	v12.2d, v16.2d, v27.d[0]
//	fmla	v13.2d, v17.2d, v27.d[0]
	prfm	PLDL1KEEP, [x9, #(0+1*128)]
//	fmla	v14.2d, v16.2d, v27.d[1]
//	fmla	v15.2d, v17.2d, v27.d[1]
	prfm	PLDL1KEEP, [x9, #(64+1*128)]

	// unroll 1
	fmla	v0.2d, v18.2d, v28.d[0]
	fmla	v1.2d, v19.2d, v28.d[0]
	ldp		q24, q25, [x10, #0] // B
	fmla	v2.2d, v18.2d, v28.d[1]
	fmla	v3.2d, v19.2d, v28.d[1]
//	ldp		q26, q27, [x10, #32] // B
	ldr		q26, [x10, #32] // B
	fmla	v4.2d, v18.2d, v29.d[0]
	fmla	v5.2d, v19.2d, v29.d[0]
	add		x10, x10, x11
	fmla	v6.2d, v18.2d, v29.d[1]
	fmla	v7.2d, v19.2d, v29.d[1]
	prfm	PLDL1KEEP, [x10, x12]
	fmla	v8.2d, v18.2d, v30.d[0]
	fmla	v9.2d, v19.2d, v30.d[0]
	add		x9, x9, #128
	fmla	v10.2d, v18.2d, v30.d[1]
	fmla	v11.2d, v19.2d, v30.d[1]
//	fmla	v12.2d, v18.2d, v31.d[0]
//	fmla	v13.2d, v19.2d, v31.d[0]
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]

	// unroll 2
	fmla	v0.2d, v20.2d, v24.d[0]
	fmla	v1.2d, v21.2d, v24.d[0]
	ldp		q28, q29, [x10, #0] // B
	fmla	v2.2d, v20.2d, v24.d[1]
	fmla	v3.2d, v21.2d, v24.d[1]
//	ldp		q30, q31, [x10, #32] // B
	ldr		q30, [x10, #32] // B
	fmla	v4.2d, v20.2d, v25.d[0]
	fmla	v5.2d, v21.2d, v25.d[0]
	add		x10, x10, x11
	fmla	v6.2d, v20.2d, v25.d[1]
	fmla	v7.2d, v21.2d, v25.d[1]
	prfm	PLDL1KEEP, [x10, x12]
	fmla	v8.2d, v20.2d, v26.d[0]
	fmla	v9.2d, v21.2d, v26.d[0]
	fmla	v10.2d, v20.2d, v26.d[1]
	fmla	v11.2d, v21.2d, v26.d[1]
	sub		w8, w8, #4
//	fmla	v12.2d, v20.2d, v27.d[0]
//	fmla	v13.2d, v21.2d, v27.d[0]
	ldp		q24, q25, [x10, #0] // B
//	fmla	v14.2d, v20.2d, v27.d[1]
//	fmla	v15.2d, v21.2d, v27.d[1]
//	ldp		q26, q27, [x10, #32] // B
	ldr		q26, [x10, #32] // B

	// unroll 3
	fmla	v0.2d, v22.2d, v28.d[0]
	fmla	v1.2d, v23.2d, v28.d[0]
	add		x10, x10, x11
	fmla	v2.2d, v22.2d, v28.d[1]
	fmla	v3.2d, v23.2d, v28.d[1]
	ldp		q16, q17, [x9, #(0*8+0*32)] // A
	fmla	v4.2d, v22.2d, v29.d[0]
	fmla	v5.2d, v23.2d, v29.d[0]
	prfm	PLDL1KEEP, [x10, x12]
	fmla	v6.2d, v22.2d, v29.d[1]
	fmla	v7.2d, v23.2d, v29.d[1]
	ldp		q18, q19, [x9, #(0*8+1*32)] // A
	fmla	v8.2d, v22.2d, v30.d[0]
	fmla	v9.2d, v23.2d, v30.d[0]
	cmp		w8, #4
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
	ldp		q20, q21, [x9, #(0*8+2*32)] // A
//	fmla	v12.2d, v22.2d, v31.d[0]
//	fmla	v13.2d, v23.2d, v31.d[0]
//	fmla	v14.2d, v22.2d, v31.d[1]
//	fmla	v15.2d, v23.2d, v31.d[1]
	ldp		q22, q23, [x9, #(0*8+3*32)] // A


	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q28, q29, [x10, #0] // B
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	ldp		q30, q31, [x10, #32] // B
	ldr		q30, [x10, #32] // B
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	add		x10, x10, x11
//	prfm	PLDL1KEEP, [x10, #(0+1*128)]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x12, #(0+1*128)]
	fmla	v8.2d, v16.2d, v26.d[0]
	fmla	v9.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x10, #(64+1*128)]
	fmla	v10.2d, v16.2d, v26.d[1]
	fmla	v11.2d, v17.2d, v26.d[1]
//	prfm	PLDL1KEEP, [x12, #(64+1*128)]
//	fmla	v12.2d, v16.2d, v27.d[0]
//	fmla	v13.2d, v17.2d, v27.d[0]
//	prfm	PLDL1KEEP, [x9, #(0+1*128)]
//	fmla	v14.2d, v16.2d, v27.d[1]
//	fmla	v15.2d, v17.2d, v27.d[1]
//	prfm	PLDL1KEEP, [x9, #(64+1*128)]

	// unroll 1
	fmla	v0.2d, v18.2d, v28.d[0]
	fmla	v1.2d, v19.2d, v28.d[0]
	ldp		q24, q25, [x10, #0] // B
	fmla	v2.2d, v18.2d, v28.d[1]
	fmla	v3.2d, v19.2d, v28.d[1]
//	ldp		q26, q27, [x10, #32] // B
	ldr		q26, [x10, #32] // B
	fmla	v4.2d, v18.2d, v29.d[0]
	fmla	v5.2d, v19.2d, v29.d[0]
	add		x10, x10, x11
	fmla	v6.2d, v18.2d, v29.d[1]
	fmla	v7.2d, v19.2d, v29.d[1]
	fmla	v8.2d, v18.2d, v30.d[0]
	fmla	v9.2d, v19.2d, v30.d[0]
	add		x9, x9, #128
	fmla	v10.2d, v18.2d, v30.d[1]
	fmla	v11.2d, v19.2d, v30.d[1]
//	fmla	v12.2d, v18.2d, v31.d[0]
//	fmla	v13.2d, v19.2d, v31.d[0]
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]

	// unroll 2
	fmla	v0.2d, v20.2d, v24.d[0]
	fmla	v1.2d, v21.2d, v24.d[0]
	ldp		q28, q29, [x10, #0] // B
	fmla	v2.2d, v20.2d, v24.d[1]
	fmla	v3.2d, v21.2d, v24.d[1]
//	ldp		q30, q31, [x10, #32] // B
	ldr		q30, [x10, #32] // B
	fmla	v4.2d, v20.2d, v25.d[0]
	fmla	v5.2d, v21.2d, v25.d[0]
	add		x10, x10, x11
	fmla	v6.2d, v20.2d, v25.d[1]
	fmla	v7.2d, v21.2d, v25.d[1]
	add		x12, x12, #128
	fmla	v8.2d, v20.2d, v26.d[0]
	fmla	v9.2d, v21.2d, v26.d[0]
	fmla	v10.2d, v20.2d, v26.d[1]
	fmla	v11.2d, v21.2d, v26.d[1]
	sub		w8, w8, #4
//	fmla	v12.2d, v20.2d, v27.d[0]
//	fmla	v13.2d, v21.2d, v27.d[0]
//	ldp		q24, q25, [x10, #0] // B
//	fmla	v14.2d, v20.2d, v27.d[1]
//	fmla	v15.2d, v21.2d, v27.d[1]
//	ldp		q26, q27, [x12, #32] // B

	// unroll 3
	fmla	v0.2d, v22.2d, v28.d[0]
	fmla	v1.2d, v23.2d, v28.d[0]
//	add		x10, x10, x11
	cmp		w8, #4
	fmla	v2.2d, v22.2d, v28.d[1]
	fmla	v3.2d, v23.2d, v28.d[1]
//	ldp		q16, q17, [x9, #(0*8+0*32)] // A
	fmla	v4.2d, v22.2d, v29.d[0]
	fmla	v5.2d, v23.2d, v29.d[0]
	fmla	v6.2d, v22.2d, v29.d[1]
	fmla	v7.2d, v23.2d, v29.d[1]
//	ldp		q18, q19, [x9, #(0*8+1*32)] // A
	fmla	v8.2d, v22.2d, v30.d[0]
	fmla	v9.2d, v23.2d, v30.d[0]
	fmla	v10.2d, v22.2d, v30.d[1]
	fmla	v11.2d, v23.2d, v30.d[1]
//	ldp		q20, q21, [x9, #(0*8+2*32)] // A
//	fmla	v12.2d, v22.2d, v31.d[0]
//	fmla	v13.2d, v23.2d, v31.d[0]
//	fmla	v14.2d, v22.2d, v31.d[1]
//	fmla	v15.2d, v23.2d, v31.d[1]
//	ldp		q22, q23, [x9, #(0*8+3*32)] // A

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x9, x9, #32
//	sub		x11, x11, #32
//	sub		x12, x12, #32
	sub		x10, x10, x11

3: // clean1-up loop

	// unroll 0
	ld1		{v16.2d, v17.2d}, [x9], #32
	ldp		q24, q25, [x10]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
//	ldp		q26, q27, [x10, #32]
	ldr		q26, [x10, #32]
	fmla	v8.2d, v16.2d, v26.d[0]
	fmla	v9.2d, v17.2d, v26.d[0]
	sub		w8, w8, #1
	fmla	v10.2d, v16.2d, v26.d[1]
	fmla	v11.2d, v17.2d, v26.d[1]
	add		x10, x10, x11
//	fmla	v12.2d, v16.2d, v27.d[0]
//	fmla	v13.2d, v17.2d, v27.d[0]
	cmp		w8, #0
//	fmla	v14.2d, v16.2d, v27.d[1]
//	fmla	v15.2d, v17.2d, v27.d[1]

	bgt		3b

2: // return



#endif



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_4x6_lib4c)
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
	.macro INNER_KERNEL_GEMM_ADD_NT_4X5_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_4x5_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x11, x11
	add		x13, x12, x11

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// main loop
1:
	
	// unroll 0
	ldp		q24, q25, [x10, #0]
//	ldp		q26, q27, [x10, #32]
	ldr		q26, [x10, #32]
	add		x10, x10, x11
	ldp		q28, q29, [x10, #0] // B
//	ldp		q30, q31, [x10, #32] // B
	ldr		q30, [x10, #32] // B
	add		x10, x10, x11

	ldp		q16, q17, [x9, #(0*8+0*32)]
	ldp		q18, q19, [x9, #(0*8+1*32)]
	ldp		q20, q21, [x9, #(0*8+2*32)]
	ldp		q22, q23, [x9, #(0*8+3*32)]
	add		x9, x9, #128

	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	fmla	v8.2d, v16.2d, v26.d[0]
	fmla	v9.2d, v17.2d, v26.d[0]
//	fmla	v10.2d, v16.2d, v26.d[1]
//	fmla	v11.2d, v17.2d, v26.d[1]
//	fmla	v12.2d, v16.2d, v27.d[0]
//	fmla	v13.2d, v17.2d, v27.d[0]
//	fmla	v14.2d, v16.2d, v27.d[1]
//	fmla	v15.2d, v17.2d, v27.d[1]

	// unroll 1
	fmla	v0.2d, v18.2d, v28.d[0]
	fmla	v1.2d, v19.2d, v28.d[0]
	fmla	v2.2d, v18.2d, v28.d[1]
	fmla	v3.2d, v19.2d, v28.d[1]
	fmla	v4.2d, v18.2d, v29.d[0]
	fmla	v5.2d, v19.2d, v29.d[0]
	fmla	v6.2d, v18.2d, v29.d[1]
	fmla	v7.2d, v19.2d, v29.d[1]
	fmla	v8.2d, v18.2d, v30.d[0]
	fmla	v9.2d, v19.2d, v30.d[0]
//	fmla	v10.2d, v18.2d, v30.d[1]
//	fmla	v11.2d, v19.2d, v30.d[1]
//	fmla	v12.2d, v18.2d, v31.d[0]
//	fmla	v13.2d, v19.2d, v31.d[0]
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]

	// unroll 2
	ldp		q24, q25, [x10, #0] // B
//	ldp		q26, q27, [x10, #32] // B
	ldr		q26, [x10, #32] // B
	add		x10, x10, x11
	ldp		q28, q29, [x10, #0] // B
//	ldp		q30, q31, [x10, #32] // B
	ldr		q30, [x10, #32] // B
	add		x10, x10, x11

	fmla	v0.2d, v20.2d, v24.d[0]
	fmla	v1.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v20.2d, v24.d[1]
	fmla	v3.2d, v21.2d, v24.d[1]
	fmla	v4.2d, v20.2d, v25.d[0]
	fmla	v5.2d, v21.2d, v25.d[0]
	fmla	v6.2d, v20.2d, v25.d[1]
	fmla	v7.2d, v21.2d, v25.d[1]
	fmla	v8.2d, v20.2d, v26.d[0]
	fmla	v9.2d, v21.2d, v26.d[0]
//	fmla	v10.2d, v20.2d, v26.d[1]
//	fmla	v11.2d, v21.2d, v26.d[1]
//	fmla	v12.2d, v20.2d, v27.d[0]
//	fmla	v13.2d, v21.2d, v27.d[0]
//	fmla	v14.2d, v20.2d, v27.d[1]
//	fmla	v15.2d, v21.2d, v27.d[1]

	// unroll 3
	fmla	v0.2d, v22.2d, v28.d[0]
	fmla	v1.2d, v23.2d, v28.d[0]
	fmla	v2.2d, v22.2d, v28.d[1]
	fmla	v3.2d, v23.2d, v28.d[1]
	fmla	v4.2d, v22.2d, v29.d[0]
	fmla	v5.2d, v23.2d, v29.d[0]
	fmla	v6.2d, v22.2d, v29.d[1]
	fmla	v7.2d, v23.2d, v29.d[1]
	fmla	v8.2d, v22.2d, v30.d[0]
	fmla	v9.2d, v23.2d, v30.d[0]
//	fmla	v10.2d, v22.2d, v30.d[1]
//	fmla	v11.2d, v23.2d, v30.d[1]
//	fmla	v12.2d, v22.2d, v31.d[0]
//	fmla	v13.2d, v23.2d, v31.d[0]
//	fmla	v14.2d, v22.2d, v31.d[1]
//	fmla	v15.2d, v23.2d, v31.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	ldp		q24, q25, [x10, #0]
//	ldp		q26, q27, [x10, #32]
	ldr		q26, [x10, #32]
	add		x10, x10, x11
	ldp		q28, q29, [x10, #0] // B
//	ldp		q30, q31, [x10, #32] // B
	ldr		q30, [x10, #32] // B
	add		x10, x10, x11

	ldp		q16, q17, [x9, #(0*8+0*32)]
	ldp		q18, q19, [x9, #(0*8+1*32)]
	ldp		q20, q21, [x9, #(0*8+2*32)]
	ldp		q22, q23, [x9, #(0*8+3*32)]
	add		x9, x9, #128

	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	fmla	v8.2d, v16.2d, v26.d[0]
	fmla	v9.2d, v17.2d, v26.d[0]
//	fmla	v10.2d, v16.2d, v26.d[1]
//	fmla	v11.2d, v17.2d, v26.d[1]
//	fmla	v12.2d, v16.2d, v27.d[0]
//	fmla	v13.2d, v17.2d, v27.d[0]
//	fmla	v14.2d, v16.2d, v27.d[1]
//	fmla	v15.2d, v17.2d, v27.d[1]

	// unroll 1
	fmla	v0.2d, v18.2d, v28.d[0]
	fmla	v1.2d, v19.2d, v28.d[0]
	fmla	v2.2d, v18.2d, v28.d[1]
	fmla	v3.2d, v19.2d, v28.d[1]
	fmla	v4.2d, v18.2d, v29.d[0]
	fmla	v5.2d, v19.2d, v29.d[0]
	fmla	v6.2d, v18.2d, v29.d[1]
	fmla	v7.2d, v19.2d, v29.d[1]
	fmla	v8.2d, v18.2d, v30.d[0]
	fmla	v9.2d, v19.2d, v30.d[0]
//	fmla	v10.2d, v18.2d, v30.d[1]
//	fmla	v11.2d, v19.2d, v30.d[1]
//	fmla	v12.2d, v18.2d, v31.d[0]
//	fmla	v13.2d, v19.2d, v31.d[0]
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]

	// unroll 2
	ldp		q24, q25, [x10, #0] // B
//	ldp		q26, q27, [x10, #32] // B
	ldr		q26, [x10, #32] // B
	add		x10, x10, x11
	ldp		q28, q29, [x10, #0] // B
//	ldp		q30, q31, [x10, #32] // B
	ldr		q30, [x10, #32] // B
	add		x10, x10, x11

	fmla	v0.2d, v20.2d, v24.d[0]
	fmla	v1.2d, v21.2d, v24.d[0]
	fmla	v2.2d, v20.2d, v24.d[1]
	fmla	v3.2d, v21.2d, v24.d[1]
	fmla	v4.2d, v20.2d, v25.d[0]
	fmla	v5.2d, v21.2d, v25.d[0]
	fmla	v6.2d, v20.2d, v25.d[1]
	fmla	v7.2d, v21.2d, v25.d[1]
	fmla	v8.2d, v20.2d, v26.d[0]
	fmla	v9.2d, v21.2d, v26.d[0]
//	fmla	v10.2d, v20.2d, v26.d[1]
//	fmla	v11.2d, v21.2d, v26.d[1]
//	fmla	v12.2d, v20.2d, v27.d[0]
//	fmla	v13.2d, v21.2d, v27.d[0]
//	fmla	v14.2d, v20.2d, v27.d[1]
//	fmla	v15.2d, v21.2d, v27.d[1]

	// unroll 3
	fmla	v0.2d, v22.2d, v28.d[0]
	fmla	v1.2d, v23.2d, v28.d[0]
	fmla	v2.2d, v22.2d, v28.d[1]
	fmla	v3.2d, v23.2d, v28.d[1]
	fmla	v4.2d, v22.2d, v29.d[0]
	fmla	v5.2d, v23.2d, v29.d[0]
	fmla	v6.2d, v22.2d, v29.d[1]
	fmla	v7.2d, v23.2d, v29.d[1]
	fmla	v8.2d, v22.2d, v30.d[0]
	fmla	v9.2d, v23.2d, v30.d[0]
//	fmla	v10.2d, v22.2d, v30.d[1]
//	fmla	v11.2d, v23.2d, v30.d[1]
//	fmla	v12.2d, v22.2d, v31.d[0]
//	fmla	v13.2d, v23.2d, v31.d[0]
//	fmla	v14.2d, v22.2d, v31.d[1]
//	fmla	v15.2d, v23.2d, v31.d[1]

	sub		w8, w8, #4
	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x9, x9, #32
//	sub		x11, x11, #32
//	sub		x12, x12, #32
//	sub		x10, x10, x11

3: // clean1-up loop

	// unroll 0
	ld1		{v16.2d, v17.2d}, [x9], #32
	ldp		q24, q25, [x10]
//	ldp		q26, q27, [x10, #32]
	ldr		q26, [x10, #32]
	add		x10, x10, x11

	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
	fmla	v8.2d, v16.2d, v26.d[0]
	fmla	v9.2d, v17.2d, v26.d[0]
	sub		w8, w8, #1
//	fmla	v10.2d, v16.2d, v26.d[1]
//	fmla	v11.2d, v17.2d, v26.d[1]
//	fmla	v12.2d, v16.2d, v27.d[0]
//	fmla	v13.2d, v17.2d, v27.d[0]
	cmp		w8, #0
//	fmla	v14.2d, v16.2d, v27.d[1]
//	fmla	v15.2d, v17.2d, v27.d[1]

	bgt		3b

2: // return



#else



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x11, x11
	add		x13, x12, x11

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x10, #0] // 0
	prfm	PLDL1KEEP, [x10, x11] // 1
	prfm	PLDL1KEEP, [x10, x12] // 2
	prfm	PLDL1KEEP, [x10, x13] // 3

	// preload
	ldp		q24, q25, [x10, #0]
//	ldp		q26, q27, [x10, #32]
	ldr		q26, [x10, #32]
	add		x10, x10, x11

//	ldp		q28, q29, [x10, #(0*8+1*32)]
//	ldp		q30, q31, [x12, #(0*8+1*32)]

	ldp		q16, q17, [x9, #(0*8+0*32)]
	ldp		q18, q19, [x9, #(0*8+1*32)]
	ldp		q20, q21, [x9, #(0*8+2*32)]
	ldp		q22, q23, [x9, #(0*8+3*32)]

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// main loop
1:
	
	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q28, q29, [x10, #0] // B
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	ldp		q30, q31, [x10, #32] // B
	ldr		q30, [x10, #32] // B
	fmla	v4.2d, v16.2d, v25.d[0]
	add		x10, x10, x11
	fmla	v5.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x10, x12]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x12, #(0+1*128)]
	fmla	v8.2d, v16.2d, v26.d[0]
	fmla	v9.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x10, #(64+1*128)]
//	fmla	v10.2d, v16.2d, v26.d[1]
//	fmla	v11.2d, v17.2d, v26.d[1]
//	prfm	PLDL1KEEP, [x12, #(64+1*128)]
//	fmla	v12.2d, v16.2d, v27.d[0]
//	fmla	v13.2d, v17.2d, v27.d[0]
	prfm	PLDL1KEEP, [x9, #(0+1*128)]
//	fmla	v14.2d, v16.2d, v27.d[1]
//	fmla	v15.2d, v17.2d, v27.d[1]
	prfm	PLDL1KEEP, [x9, #(64+1*128)]

	// unroll 1
	fmla	v0.2d, v18.2d, v28.d[0]
	fmla	v1.2d, v19.2d, v28.d[0]
	ldp		q24, q25, [x10, #0] // B
	fmla	v2.2d, v18.2d, v28.d[1]
	fmla	v3.2d, v19.2d, v28.d[1]
//	ldp		q26, q27, [x10, #32] // B
	ldr		q26, [x10, #32] // B
	fmla	v4.2d, v18.2d, v29.d[0]
	fmla	v5.2d, v19.2d, v29.d[0]
	add		x10, x10, x11
	fmla	v6.2d, v18.2d, v29.d[1]
	fmla	v7.2d, v19.2d, v29.d[1]
	prfm	PLDL1KEEP, [x10, x12]
	fmla	v8.2d, v18.2d, v30.d[0]
	fmla	v9.2d, v19.2d, v30.d[0]
	add		x9, x9, #128
//	fmla	v10.2d, v18.2d, v30.d[1]
//	fmla	v11.2d, v19.2d, v30.d[1]
//	fmla	v12.2d, v18.2d, v31.d[0]
//	fmla	v13.2d, v19.2d, v31.d[0]
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]

	// unroll 2
	fmla	v0.2d, v20.2d, v24.d[0]
	fmla	v1.2d, v21.2d, v24.d[0]
	ldp		q28, q29, [x10, #0] // B
	fmla	v2.2d, v20.2d, v24.d[1]
	fmla	v3.2d, v21.2d, v24.d[1]
//	ldp		q30, q31, [x10, #32] // B
	ldr		q30, [x10, #32] // B
	fmla	v4.2d, v20.2d, v25.d[0]
	fmla	v5.2d, v21.2d, v25.d[0]
	add		x10, x10, x11
	fmla	v6.2d, v20.2d, v25.d[1]
	fmla	v7.2d, v21.2d, v25.d[1]
	prfm	PLDL1KEEP, [x10, x12]
	fmla	v8.2d, v20.2d, v26.d[0]
	fmla	v9.2d, v21.2d, v26.d[0]
//	fmla	v10.2d, v20.2d, v26.d[1]
//	fmla	v11.2d, v21.2d, v26.d[1]
	sub		w8, w8, #4
//	fmla	v12.2d, v20.2d, v27.d[0]
//	fmla	v13.2d, v21.2d, v27.d[0]
	ldp		q24, q25, [x10, #0] // B
//	fmla	v14.2d, v20.2d, v27.d[1]
//	fmla	v15.2d, v21.2d, v27.d[1]
//	ldp		q26, q27, [x10, #32] // B
	ldr		q26, [x10, #32] // B

	// unroll 3
	fmla	v0.2d, v22.2d, v28.d[0]
	fmla	v1.2d, v23.2d, v28.d[0]
	add		x10, x10, x11
	fmla	v2.2d, v22.2d, v28.d[1]
	fmla	v3.2d, v23.2d, v28.d[1]
	ldp		q16, q17, [x9, #(0*8+0*32)] // A
	fmla	v4.2d, v22.2d, v29.d[0]
	fmla	v5.2d, v23.2d, v29.d[0]
	prfm	PLDL1KEEP, [x10, x12]
	fmla	v6.2d, v22.2d, v29.d[1]
	fmla	v7.2d, v23.2d, v29.d[1]
	ldp		q18, q19, [x9, #(0*8+1*32)] // A
	fmla	v8.2d, v22.2d, v30.d[0]
	fmla	v9.2d, v23.2d, v30.d[0]
	cmp		w8, #4
//	fmla	v10.2d, v22.2d, v30.d[1]
//	fmla	v11.2d, v23.2d, v30.d[1]
	ldp		q20, q21, [x9, #(0*8+2*32)] // A
//	fmla	v12.2d, v22.2d, v31.d[0]
//	fmla	v13.2d, v23.2d, v31.d[0]
//	fmla	v14.2d, v22.2d, v31.d[1]
//	fmla	v15.2d, v23.2d, v31.d[1]
	ldp		q22, q23, [x9, #(0*8+3*32)] // A


	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	ldp		q28, q29, [x10, #0] // B
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	ldp		q30, q31, [x10, #32] // B
	ldr		q30, [x10, #32] // B
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	add		x10, x10, x11
//	prfm	PLDL1KEEP, [x10, #(0+1*128)]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x12, #(0+1*128)]
	fmla	v8.2d, v16.2d, v26.d[0]
	fmla	v9.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x10, #(64+1*128)]
//	fmla	v10.2d, v16.2d, v26.d[1]
//	fmla	v11.2d, v17.2d, v26.d[1]
//	prfm	PLDL1KEEP, [x12, #(64+1*128)]
//	fmla	v12.2d, v16.2d, v27.d[0]
//	fmla	v13.2d, v17.2d, v27.d[0]
//	prfm	PLDL1KEEP, [x9, #(0+1*128)]
//	fmla	v14.2d, v16.2d, v27.d[1]
//	fmla	v15.2d, v17.2d, v27.d[1]
//	prfm	PLDL1KEEP, [x9, #(64+1*128)]

	// unroll 1
	fmla	v0.2d, v18.2d, v28.d[0]
	fmla	v1.2d, v19.2d, v28.d[0]
	ldp		q24, q25, [x10, #0] // B
	fmla	v2.2d, v18.2d, v28.d[1]
	fmla	v3.2d, v19.2d, v28.d[1]
//	ldp		q26, q27, [x10, #32] // B
	ldr		q26, [x10, #32] // B
	fmla	v4.2d, v18.2d, v29.d[0]
	fmla	v5.2d, v19.2d, v29.d[0]
	add		x10, x10, x11
	fmla	v6.2d, v18.2d, v29.d[1]
	fmla	v7.2d, v19.2d, v29.d[1]
	fmla	v8.2d, v18.2d, v30.d[0]
	fmla	v9.2d, v19.2d, v30.d[0]
	add		x9, x9, #128
//	fmla	v10.2d, v18.2d, v30.d[1]
//	fmla	v11.2d, v19.2d, v30.d[1]
//	fmla	v12.2d, v18.2d, v31.d[0]
//	fmla	v13.2d, v19.2d, v31.d[0]
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]

	// unroll 2
	fmla	v0.2d, v20.2d, v24.d[0]
	fmla	v1.2d, v21.2d, v24.d[0]
	ldp		q28, q29, [x10, #0] // B
	fmla	v2.2d, v20.2d, v24.d[1]
	fmla	v3.2d, v21.2d, v24.d[1]
//	ldp		q30, q31, [x10, #32] // B
	ldr		q30, [x10, #32] // B
	fmla	v4.2d, v20.2d, v25.d[0]
	fmla	v5.2d, v21.2d, v25.d[0]
	add		x10, x10, x11
	fmla	v6.2d, v20.2d, v25.d[1]
	fmla	v7.2d, v21.2d, v25.d[1]
	add		x12, x12, #128
	fmla	v8.2d, v20.2d, v26.d[0]
	fmla	v9.2d, v21.2d, v26.d[0]
//	fmla	v10.2d, v20.2d, v26.d[1]
//	fmla	v11.2d, v21.2d, v26.d[1]
	sub		w8, w8, #4
//	fmla	v12.2d, v20.2d, v27.d[0]
//	fmla	v13.2d, v21.2d, v27.d[0]
//	ldp		q24, q25, [x10, #0] // B
//	fmla	v14.2d, v20.2d, v27.d[1]
//	fmla	v15.2d, v21.2d, v27.d[1]
//	ldp		q26, q27, [x12, #32] // B

	// unroll 3
	fmla	v0.2d, v22.2d, v28.d[0]
	fmla	v1.2d, v23.2d, v28.d[0]
//	add		x10, x10, x11
	cmp		w8, #4
	fmla	v2.2d, v22.2d, v28.d[1]
	fmla	v3.2d, v23.2d, v28.d[1]
//	ldp		q16, q17, [x9, #(0*8+0*32)] // A
	fmla	v4.2d, v22.2d, v29.d[0]
	fmla	v5.2d, v23.2d, v29.d[0]
	fmla	v6.2d, v22.2d, v29.d[1]
	fmla	v7.2d, v23.2d, v29.d[1]
//	ldp		q18, q19, [x9, #(0*8+1*32)] // A
	fmla	v8.2d, v22.2d, v30.d[0]
	fmla	v9.2d, v23.2d, v30.d[0]
//	fmla	v10.2d, v22.2d, v30.d[1]
//	fmla	v11.2d, v23.2d, v30.d[1]
//	ldp		q20, q21, [x9, #(0*8+2*32)] // A
//	fmla	v12.2d, v22.2d, v31.d[0]
//	fmla	v13.2d, v23.2d, v31.d[0]
//	fmla	v14.2d, v22.2d, v31.d[1]
//	fmla	v15.2d, v23.2d, v31.d[1]
//	ldp		q22, q23, [x9, #(0*8+3*32)] // A

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x9, x9, #32
//	sub		x11, x11, #32
//	sub		x12, x12, #32
	sub		x10, x10, x11

3: // clean1-up loop

	// unroll 0
	ld1		{v16.2d, v17.2d}, [x9], #32
	ldp		q24, q25, [x10]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]
//	ldp		q26, q27, [x10, #32]
	ldr		q26, [x10, #32]
	fmla	v8.2d, v16.2d, v26.d[0]
	fmla	v9.2d, v17.2d, v26.d[0]
	sub		w8, w8, #1
//	fmla	v10.2d, v16.2d, v26.d[1]
//	fmla	v11.2d, v17.2d, v26.d[1]
	add		x10, x10, x11
//	fmla	v12.2d, v16.2d, v27.d[0]
//	fmla	v13.2d, v17.2d, v27.d[0]
	cmp		w8, #0
//	fmla	v14.2d, v16.2d, v27.d[1]
//	fmla	v15.2d, v17.2d, v27.d[1]

	bgt		3b

2: // return



#endif



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_4x5_lib4c)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_4X8_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_4x8_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x11, x11

	add		x13, x10, x11 // 1
	add		x14, x10, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5
	add		x18, x16, x20 // 6
	add		x19, x17, x20 // 7

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x10], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q31, [x19], #16
	ldp		q16, q17, [x9, #0]
	ldp		q18, q19, [x9, #32]

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x10], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q31, [x19], #16
	ldp		q16, q17, [x9, #64]
	ldp		q18, q19, [x9, #96]

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	add		x9, x9, #128
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x10], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q31, [x19], #16
	ldp		q16, q17, [x9, #0]
	ldp		q18, q19, [x9, #32]

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x10], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q31, [x19], #16
	ldp		q16, q17, [x9, #64]
	ldp		q18, q19, [x9, #96]

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	add		x9, x9, #128
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// load 0
	ldr		d24, [x10], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
	ldr		d30, [x18], #8
	ldr		d31, [x19], #8
	ldp		q16, q17, [x9]
	add		x9, x9, #32

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x11, x11

	add		x13, x10, x11 // 1
	add		x14, x10, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5
	add		x18, x16, x20 // 6
	add		x19, x17, x20 // 7

	// prefetch
	prfm	PLDL1KEEP, [x10, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x17, #0]
	prfm	PLDL1KEEP, [x18, #0]
	prfm	PLDL1KEEP, [x19, #0]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, #32]
	prfm	PLDL1KEEP, [x9, #64]

	// preload
	ldp		q20, q24, [x10], #32
	ldp		q21, q25, [x13], #32
	ldp		q22, q26, [x14], #32
	ldp		q23, q27, [x15], #32
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q31, [x19], #16
	ldp		q16, q17, [x9]

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x10, #0]
//	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x14, #0]
//	prfm	PLDL1KEEP, [x15, #0]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x9, x20]
	prfm	PLDL1KEEP, [x9, #96]

	// zero tmp acc

//	add		x13, x11, #64
//	add		x13, x11, x11
//	add		x14, x13, #64

	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #32]
	fmla	v0.2d, v16.2d, v20.d[0]
	fmla	v1.2d, v17.2d, v20.d[0]
	prfm	PLDL1KEEP, [x9, #128]
	fmla	v2.2d, v16.2d, v21.d[0]
	fmla	v3.2d, v17.2d, v21.d[0]
	prfm	PLDL1KEEP, [x10, #0]
	fmla	v4.2d, v16.2d, v22.d[0]
	prfm	PLDL1KEEP, [x13, #0]
	fmla	v5.2d, v17.2d, v22.d[0]
	prfm	PLDL1KEEP, [x14, #0]
	fmla	v6.2d, v16.2d, v23.d[0]
	prfm	PLDL1KEEP, [x15, #0]
	fmla	v7.2d, v17.2d, v23.d[0]
	prfm	PLDL1KEEP, [x16, #16]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x17, #16]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	prfm	PLDL1KEEP, [x18, #16]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
	prfm	PLDL1KEEP, [x19, #16]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 1
	ldp		q16, q17, [x9, #64]
	fmla	v0.2d, v18.2d, v20.d[1]
	fmla	v1.2d, v19.2d, v20.d[1]
//	ldr		q24, [x10], #16
	fmla	v2.2d, v18.2d, v21.d[1]
	prfm	PLDL1KEEP, [x9, #160]
	fmla	v3.2d, v19.2d, v21.d[1]
//	ldr		q25, [x13], #16
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v5.2d, v19.2d, v22.d[1]
//	ldr		q26, [x14], #16
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v7.2d, v19.2d, v23.d[1]
//	ldr		q27, [x15], #16
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	ldr		q30, [x18], #16
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]
	ldr		q31, [x19], #16

	// unroll 2
	ldp		q18, q19, [x9, #96]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, #128
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, #64]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 3
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldp		q20, q24, [x10], #32
	fmla	v2.2d, v18.2d, v25.d[1]
	prfm	PLDL1KEEP, [x9, #96]
	fmla	v3.2d, v19.2d, v25.d[1]
	ldp		q21, q25, [x13], #32
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	ldp		q22, q26, [x14], #32
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	ldp		q23, q27, [x15], #32
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	ldr		q30, [x18], #16
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]
	ldr		q31, [x19], #16

	bgt		1b


	// reduce

//	sub		x9, x9, #32
//	sub		x10, x10, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldp		q18, q19, [x9, #32]
	fmla	v0.2d, v16.2d, v20.d[0]
	fmla	v1.2d, v17.2d, v20.d[0]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
	fmla	v3.2d, v17.2d, v21.d[0]
//	prfm	PLDL1KEEP, [x10, #32]
	fmla	v4.2d, v16.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x13, #32]
	fmla	v5.2d, v17.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x14, #32]
	fmla	v6.2d, v16.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x15, #32]
	fmla	v7.2d, v17.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x16, #32]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x17, #32]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
//	prfm	PLDL1KEEP, [x18, #32]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
//	prfm	PLDL1KEEP, [x19, #32]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 1
	ldp		q16, q17, [x9, #64]
	fmla	v0.2d, v18.2d, v20.d[1]
	fmla	v1.2d, v19.2d, v20.d[1]
//	ldr		q24, [x10], #16
	fmla	v2.2d, v18.2d, v21.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v21.d[1]
//	ldr		q25, [x13], #16
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v5.2d, v19.2d, v22.d[1]
//	ldr		q26, [x14], #16
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v7.2d, v19.2d, v23.d[1]
//	ldr		q27, [x15], #16
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	ldr		q30, [x18], #16
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]
	ldr		q31, [x19], #16

	// unroll 2
	ldp		q18, q19, [x9, #96]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, #128
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
//	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 3
//	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
//	ldr		q24, [x10], #16
	fmla	v2.2d, v18.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v25.d[1]
//	ldr		q25, [x13], #16
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
//	ldr		q26, [x14], #16
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
//	ldr		q27, [x15], #16
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
//	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
//	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
//	ldr		q30, [x18], #16
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]
//	ldr		q31, [x19], #16

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x10, x10, #32
	sub		x13, x13, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x16, x16, #16
	sub		x17, x17, #16
	sub		x18, x18, #16
	sub		x19, x19, #16

3: // clean1-up loop

	// load 0
	ldr		d24, [x10], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
	ldr		d30, [x18], #8
	ldr		d31, [x19], #8
	ldp		q16, q17, [x9]
	add		x9, x9, #32

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_4x8_lib4c)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_4X7_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_4x7_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x11, x11

	add		x13, x10, x11 // 1
	add		x14, x10, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5
	add		x18, x16, x20 // 6
//	add		x19, x17, x20 // 7

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x10], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
//	ldr		q31, [x19], #16
	ldp		q16, q17, [x9, #0]
	ldp		q18, q19, [x9, #32]

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x10], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
//	ldr		q31, [x19], #16
	ldp		q16, q17, [x9, #64]
	ldp		q18, q19, [x9, #96]

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	add		x9, x9, #128
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x10], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
//	ldr		q31, [x19], #16
	ldp		q16, q17, [x9, #0]
	ldp		q18, q19, [x9, #32]

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x10], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
//	ldr		q31, [x19], #16
	ldp		q16, q17, [x9, #64]
	ldp		q18, q19, [x9, #96]

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	add		x9, x9, #128
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// load 0
	ldr		d24, [x10], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
	ldr		d30, [x18], #8
//	ldr		d31, [x19], #8
	ldp		q16, q17, [x9]
	add		x9, x9, #32

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x11, x11

	add		x13, x10, x11 // 1
	add		x14, x10, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5
	add		x18, x16, x20 // 6
//	add		x19, x17, x20 // 7

	// prefetch
	prfm	PLDL1KEEP, [x10, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x17, #0]
	prfm	PLDL1KEEP, [x18, #0]
//	prfm	PLDL1KEEP, [x19, #0]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, #32]
	prfm	PLDL1KEEP, [x9, #64]

	// preload
	ldp		q20, q24, [x10], #32
	ldp		q21, q25, [x13], #32
	ldp		q22, q26, [x14], #32
	ldp		q23, q27, [x15], #32
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
//	ldr		q31, [x19], #16
	ldp		q16, q17, [x9]

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x10, #0]
//	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x14, #0]
//	prfm	PLDL1KEEP, [x15, #0]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x9, x20]
	prfm	PLDL1KEEP, [x9, #96]

	// zero tmp acc

//	add		x13, x11, #64
//	add		x13, x11, x11
//	add		x14, x13, #64

	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #32]
	fmla	v0.2d, v16.2d, v20.d[0]
	fmla	v1.2d, v17.2d, v20.d[0]
	prfm	PLDL1KEEP, [x9, #128]
	fmla	v2.2d, v16.2d, v21.d[0]
	fmla	v3.2d, v17.2d, v21.d[0]
	prfm	PLDL1KEEP, [x10, #0]
	fmla	v4.2d, v16.2d, v22.d[0]
	prfm	PLDL1KEEP, [x13, #0]
	fmla	v5.2d, v17.2d, v22.d[0]
	prfm	PLDL1KEEP, [x14, #0]
	fmla	v6.2d, v16.2d, v23.d[0]
	prfm	PLDL1KEEP, [x15, #0]
	fmla	v7.2d, v17.2d, v23.d[0]
	prfm	PLDL1KEEP, [x16, #16]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x17, #16]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	prfm	PLDL1KEEP, [x18, #16]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
//	prfm	PLDL1KEEP, [x19, #16]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 1
	ldp		q16, q17, [x9, #64]
	fmla	v0.2d, v18.2d, v20.d[1]
	fmla	v1.2d, v19.2d, v20.d[1]
//	ldr		q24, [x10], #16
	fmla	v2.2d, v18.2d, v21.d[1]
	prfm	PLDL1KEEP, [x9, #160]
	fmla	v3.2d, v19.2d, v21.d[1]
//	ldr		q25, [x13], #16
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v5.2d, v19.2d, v22.d[1]
//	ldr		q26, [x14], #16
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v7.2d, v19.2d, v23.d[1]
//	ldr		q27, [x15], #16
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	ldr		q30, [x18], #16
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]
//	ldr		q31, [x19], #16

	// unroll 2
	ldp		q18, q19, [x9, #96]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, #128
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, #64]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 3
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldp		q20, q24, [x10], #32
	fmla	v2.2d, v18.2d, v25.d[1]
	prfm	PLDL1KEEP, [x9, #96]
	fmla	v3.2d, v19.2d, v25.d[1]
	ldp		q21, q25, [x13], #32
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	ldp		q22, q26, [x14], #32
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	ldp		q23, q27, [x15], #32
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	ldr		q30, [x18], #16
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]
//	ldr		q31, [x19], #16

	bgt		1b


	// reduce

//	sub		x9, x9, #32
//	sub		x10, x10, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldp		q18, q19, [x9, #32]
	fmla	v0.2d, v16.2d, v20.d[0]
	fmla	v1.2d, v17.2d, v20.d[0]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
	fmla	v3.2d, v17.2d, v21.d[0]
//	prfm	PLDL1KEEP, [x10, #32]
	fmla	v4.2d, v16.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x13, #32]
	fmla	v5.2d, v17.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x14, #32]
	fmla	v6.2d, v16.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x15, #32]
	fmla	v7.2d, v17.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x16, #32]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x17, #32]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
//	prfm	PLDL1KEEP, [x18, #32]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
//	prfm	PLDL1KEEP, [x19, #32]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 1
	ldp		q16, q17, [x9, #64]
	fmla	v0.2d, v18.2d, v20.d[1]
	fmla	v1.2d, v19.2d, v20.d[1]
//	ldr		q24, [x10], #16
	fmla	v2.2d, v18.2d, v21.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v21.d[1]
//	ldr		q25, [x13], #16
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v5.2d, v19.2d, v22.d[1]
//	ldr		q26, [x14], #16
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v7.2d, v19.2d, v23.d[1]
//	ldr		q27, [x15], #16
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	ldr		q30, [x18], #16
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]
//	ldr		q31, [x19], #16

	// unroll 2
	ldp		q18, q19, [x9, #96]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, #128
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
//	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 3
//	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
//	ldr		q24, [x10], #16
	fmla	v2.2d, v18.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v25.d[1]
//	ldr		q25, [x13], #16
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
//	ldr		q26, [x14], #16
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
//	ldr		q27, [x15], #16
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
//	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
//	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
//	ldr		q30, [x18], #16
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]
//	ldr		q31, [x19], #16

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x10, x10, #32
	sub		x13, x13, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x16, x16, #16
	sub		x17, x17, #16
	sub		x18, x18, #16
//	sub		x19, x19, #16

3: // clean1-up loop

	// load 0
	ldr		d24, [x10], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
	ldr		d30, [x18], #8
//	ldr		d31, [x19], #8
	ldp		q16, q17, [x9]
	add		x9, x9, #32

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_4x7_lib4c)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_4X6_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_4x6_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x11, x11

	add		x13, x10, x11 // 1
	add		x14, x10, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5
//	add		x18, x16, x20 // 6
//	add		x19, x17, x20 // 7

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x10], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
//	ldr		q30, [x18], #16
//	ldr		q31, [x19], #16
	ldp		q16, q17, [x9, #0]
	ldp		q18, q19, [x9, #32]

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
//	fmla	v12.2d, v16.2d, v30.d[0]
//	fmla	v13.2d, v17.2d, v30.d[0]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
//	fmla	v12.2d, v18.2d, v30.d[1]
//	fmla	v13.2d, v19.2d, v30.d[1]
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x10], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
//	ldr		q30, [x18], #16
//	ldr		q31, [x19], #16
	ldp		q16, q17, [x9, #64]
	ldp		q18, q19, [x9, #96]

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	add		x9, x9, #128
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
//	fmla	v12.2d, v16.2d, v30.d[0]
//	fmla	v13.2d, v17.2d, v30.d[0]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
//	fmla	v12.2d, v18.2d, v30.d[1]
//	fmla	v13.2d, v19.2d, v30.d[1]
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x10], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
//	ldr		q30, [x18], #16
//	ldr		q31, [x19], #16
	ldp		q16, q17, [x9, #0]
	ldp		q18, q19, [x9, #32]

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
//	fmla	v12.2d, v16.2d, v30.d[0]
//	fmla	v13.2d, v17.2d, v30.d[0]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
//	fmla	v12.2d, v18.2d, v30.d[1]
//	fmla	v13.2d, v19.2d, v30.d[1]
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x10], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
//	ldr		q30, [x18], #16
//	ldr		q31, [x19], #16
	ldp		q16, q17, [x9, #64]
	ldp		q18, q19, [x9, #96]

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	add		x9, x9, #128
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
//	fmla	v12.2d, v16.2d, v30.d[0]
//	fmla	v13.2d, v17.2d, v30.d[0]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
//	fmla	v12.2d, v18.2d, v30.d[1]
//	fmla	v13.2d, v19.2d, v30.d[1]
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// load 0
	ldr		d24, [x10], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
//	ldr		d30, [x18], #8
//	ldr		d31, [x19], #8
	ldp		q16, q17, [x9]
	add		x9, x9, #32

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
//	fmla	v12.2d, v16.2d, v30.d[0]
//	fmla	v13.2d, v17.2d, v30.d[0]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x11, x11

	add		x13, x10, x11 // 1
	add		x14, x10, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5
//	add		x18, x16, x20 // 6
//	add		x19, x17, x20 // 7

	// prefetch
	prfm	PLDL1KEEP, [x10, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x17, #0]
//	prfm	PLDL1KEEP, [x18, #0]
//	prfm	PLDL1KEEP, [x19, #0]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, #32]
	prfm	PLDL1KEEP, [x9, #64]

	// preload
	ldp		q20, q24, [x10], #32
	ldp		q21, q25, [x13], #32
	ldp		q22, q26, [x14], #32
	ldp		q23, q27, [x15], #32
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
//	ldr		q30, [x18], #16
//	ldr		q31, [x19], #16
	ldp		q16, q17, [x9]

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x10, #0]
//	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x14, #0]
//	prfm	PLDL1KEEP, [x15, #0]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x9, x20]
	prfm	PLDL1KEEP, [x9, #96]

	// zero tmp acc

//	add		x13, x11, #64
//	add		x13, x11, x11
//	add		x14, x13, #64

	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #32]
	fmla	v0.2d, v16.2d, v20.d[0]
	fmla	v1.2d, v17.2d, v20.d[0]
	prfm	PLDL1KEEP, [x9, #128]
	fmla	v2.2d, v16.2d, v21.d[0]
	fmla	v3.2d, v17.2d, v21.d[0]
	prfm	PLDL1KEEP, [x10, #0]
	fmla	v4.2d, v16.2d, v22.d[0]
	prfm	PLDL1KEEP, [x13, #0]
	fmla	v5.2d, v17.2d, v22.d[0]
	prfm	PLDL1KEEP, [x14, #0]
	fmla	v6.2d, v16.2d, v23.d[0]
	prfm	PLDL1KEEP, [x15, #0]
	fmla	v7.2d, v17.2d, v23.d[0]
	prfm	PLDL1KEEP, [x16, #16]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x17, #16]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
//	prfm	PLDL1KEEP, [x18, #16]
//	fmla	v12.2d, v16.2d, v30.d[0]
//	fmla	v13.2d, v17.2d, v30.d[0]
//	prfm	PLDL1KEEP, [x19, #16]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 1
	ldp		q16, q17, [x9, #64]
	fmla	v0.2d, v18.2d, v20.d[1]
	fmla	v1.2d, v19.2d, v20.d[1]
//	ldr		q24, [x10], #16
	fmla	v2.2d, v18.2d, v21.d[1]
	prfm	PLDL1KEEP, [x9, #160]
	fmla	v3.2d, v19.2d, v21.d[1]
//	ldr		q25, [x13], #16
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v5.2d, v19.2d, v22.d[1]
//	ldr		q26, [x14], #16
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v7.2d, v19.2d, v23.d[1]
//	ldr		q27, [x15], #16
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	ldr		q29, [x17], #16
//	fmla	v12.2d, v18.2d, v30.d[1]
//	fmla	v13.2d, v19.2d, v30.d[1]
//	ldr		q30, [x18], #16
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]
//	ldr		q31, [x19], #16

	// unroll 2
	ldp		q18, q19, [x9, #96]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, #128
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, #64]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
//	fmla	v12.2d, v16.2d, v30.d[0]
//	fmla	v13.2d, v17.2d, v30.d[0]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 3
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldp		q20, q24, [x10], #32
	fmla	v2.2d, v18.2d, v25.d[1]
	prfm	PLDL1KEEP, [x9, #96]
	fmla	v3.2d, v19.2d, v25.d[1]
	ldp		q21, q25, [x13], #32
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	ldp		q22, q26, [x14], #32
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	ldp		q23, q27, [x15], #32
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	ldr		q29, [x17], #16
//	fmla	v12.2d, v18.2d, v30.d[1]
//	fmla	v13.2d, v19.2d, v30.d[1]
//	ldr		q30, [x18], #16
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]
//	ldr		q31, [x19], #16

	bgt		1b


	// reduce

//	sub		x9, x9, #32
//	sub		x10, x10, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldp		q18, q19, [x9, #32]
	fmla	v0.2d, v16.2d, v20.d[0]
	fmla	v1.2d, v17.2d, v20.d[0]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
	fmla	v3.2d, v17.2d, v21.d[0]
//	prfm	PLDL1KEEP, [x10, #32]
	fmla	v4.2d, v16.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x13, #32]
	fmla	v5.2d, v17.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x14, #32]
	fmla	v6.2d, v16.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x15, #32]
	fmla	v7.2d, v17.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x16, #32]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x17, #32]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
//	prfm	PLDL1KEEP, [x18, #32]
//	fmla	v12.2d, v16.2d, v30.d[0]
//	fmla	v13.2d, v17.2d, v30.d[0]
//	prfm	PLDL1KEEP, [x19, #32]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 1
	ldp		q16, q17, [x9, #64]
	fmla	v0.2d, v18.2d, v20.d[1]
	fmla	v1.2d, v19.2d, v20.d[1]
//	ldr		q24, [x10], #16
	fmla	v2.2d, v18.2d, v21.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v21.d[1]
//	ldr		q25, [x13], #16
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v5.2d, v19.2d, v22.d[1]
//	ldr		q26, [x14], #16
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v7.2d, v19.2d, v23.d[1]
//	ldr		q27, [x15], #16
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	ldr		q29, [x17], #16
//	fmla	v12.2d, v18.2d, v30.d[1]
//	fmla	v13.2d, v19.2d, v30.d[1]
//	ldr		q30, [x18], #16
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]
//	ldr		q31, [x19], #16

	// unroll 2
	ldp		q18, q19, [x9, #96]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, #128
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
//	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
//	fmla	v12.2d, v16.2d, v30.d[0]
//	fmla	v13.2d, v17.2d, v30.d[0]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 3
//	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
//	ldr		q24, [x10], #16
	fmla	v2.2d, v18.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v25.d[1]
//	ldr		q25, [x13], #16
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
//	ldr		q26, [x14], #16
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
//	ldr		q27, [x15], #16
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
//	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
//	ldr		q29, [x17], #16
//	fmla	v12.2d, v18.2d, v30.d[1]
//	fmla	v13.2d, v19.2d, v30.d[1]
//	ldr		q30, [x18], #16
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]
//	ldr		q31, [x19], #16

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x10, x10, #32
	sub		x13, x13, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x16, x16, #16
	sub		x17, x17, #16
//	sub		x18, x18, #16
//	sub		x19, x19, #16

3: // clean1-up loop

	// load 0
	ldr		d24, [x10], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
//	ldr		d30, [x18], #8
//	ldr		d31, [x19], #8
	ldp		q16, q17, [x9]
	add		x9, x9, #32

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
//	fmla	v12.2d, v16.2d, v30.d[0]
//	fmla	v13.2d, v17.2d, v30.d[0]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_4x6_lib4c)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_4X5_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_4x5_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x11, x11

	add		x13, x10, x11 // 1
	add		x14, x10, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
//	add		x17, x15, x20 // 5
//	add		x18, x16, x20 // 6
//	add		x19, x17, x20 // 7

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x10], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
//	ldr		q29, [x17], #16
//	ldr		q30, [x18], #16
//	ldr		q31, [x19], #16
	ldp		q16, q17, [x9, #0]
	ldp		q18, q19, [x9, #32]

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
//	fmla	v10.2d, v16.2d, v29.d[0]
//	fmla	v11.2d, v17.2d, v29.d[0]
//	fmla	v12.2d, v16.2d, v30.d[0]
//	fmla	v13.2d, v17.2d, v30.d[0]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
//	fmla	v10.2d, v18.2d, v29.d[1]
//	fmla	v11.2d, v19.2d, v29.d[1]
//	fmla	v12.2d, v18.2d, v30.d[1]
//	fmla	v13.2d, v19.2d, v30.d[1]
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x10], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
//	ldr		q29, [x17], #16
//	ldr		q30, [x18], #16
//	ldr		q31, [x19], #16
	ldp		q16, q17, [x9, #64]
	ldp		q18, q19, [x9, #96]

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	add		x9, x9, #128
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
//	fmla	v10.2d, v16.2d, v29.d[0]
//	fmla	v11.2d, v17.2d, v29.d[0]
//	fmla	v12.2d, v16.2d, v30.d[0]
//	fmla	v13.2d, v17.2d, v30.d[0]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
//	fmla	v10.2d, v18.2d, v29.d[1]
//	fmla	v11.2d, v19.2d, v29.d[1]
//	fmla	v12.2d, v18.2d, v30.d[1]
//	fmla	v13.2d, v19.2d, v30.d[1]
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x10], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
//	ldr		q29, [x17], #16
//	ldr		q30, [x18], #16
//	ldr		q31, [x19], #16
	ldp		q16, q17, [x9, #0]
	ldp		q18, q19, [x9, #32]

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
//	fmla	v10.2d, v16.2d, v29.d[0]
//	fmla	v11.2d, v17.2d, v29.d[0]
//	fmla	v12.2d, v16.2d, v30.d[0]
//	fmla	v13.2d, v17.2d, v30.d[0]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
//	fmla	v10.2d, v18.2d, v29.d[1]
//	fmla	v11.2d, v19.2d, v29.d[1]
//	fmla	v12.2d, v18.2d, v30.d[1]
//	fmla	v13.2d, v19.2d, v30.d[1]
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x10], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
//	ldr		q29, [x17], #16
//	ldr		q30, [x18], #16
//	ldr		q31, [x19], #16
	ldp		q16, q17, [x9, #64]
	ldp		q18, q19, [x9, #96]

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	add		x9, x9, #128
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
//	fmla	v10.2d, v16.2d, v29.d[0]
//	fmla	v11.2d, v17.2d, v29.d[0]
//	fmla	v12.2d, v16.2d, v30.d[0]
//	fmla	v13.2d, v17.2d, v30.d[0]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
//	fmla	v10.2d, v18.2d, v29.d[1]
//	fmla	v11.2d, v19.2d, v29.d[1]
//	fmla	v12.2d, v18.2d, v30.d[1]
//	fmla	v13.2d, v19.2d, v30.d[1]
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// load 0
	ldr		d24, [x10], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
//	ldr		d29, [x17], #8
//	ldr		d30, [x18], #8
//	ldr		d31, [x19], #8
	ldp		q16, q17, [x9]
	add		x9, x9, #32

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
//	fmla	v10.2d, v16.2d, v29.d[0]
//	fmla	v11.2d, v17.2d, v29.d[0]
//	fmla	v12.2d, v16.2d, v30.d[0]
//	fmla	v13.2d, v17.2d, v30.d[0]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x11, x11

	add		x13, x10, x11 // 1
	add		x14, x10, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
//	add		x17, x15, x20 // 5
//	add		x18, x16, x20 // 6
//	add		x19, x17, x20 // 7

	// prefetch
	prfm	PLDL1KEEP, [x10, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #0]
//	prfm	PLDL1KEEP, [x17, #0]
//	prfm	PLDL1KEEP, [x18, #0]
//	prfm	PLDL1KEEP, [x19, #0]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, #32]
	prfm	PLDL1KEEP, [x9, #64]

	// preload
	ldp		q20, q24, [x10], #32
	ldp		q21, q25, [x13], #32
	ldp		q22, q26, [x14], #32
	ldp		q23, q27, [x15], #32
	ldr		q28, [x16], #16
//	ldr		q29, [x17], #16
//	ldr		q30, [x18], #16
//	ldr		q31, [x19], #16
	ldp		q16, q17, [x9]

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x10, #0]
//	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x14, #0]
//	prfm	PLDL1KEEP, [x15, #0]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x9, x20]
	prfm	PLDL1KEEP, [x9, #96]

	// zero tmp acc

//	add		x13, x11, #64
//	add		x13, x11, x11
//	add		x14, x13, #64

	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #32]
	fmla	v0.2d, v16.2d, v20.d[0]
	fmla	v1.2d, v17.2d, v20.d[0]
	prfm	PLDL1KEEP, [x9, #128]
	fmla	v2.2d, v16.2d, v21.d[0]
	fmla	v3.2d, v17.2d, v21.d[0]
	prfm	PLDL1KEEP, [x10, #0]
	fmla	v4.2d, v16.2d, v22.d[0]
	prfm	PLDL1KEEP, [x13, #0]
	fmla	v5.2d, v17.2d, v22.d[0]
	prfm	PLDL1KEEP, [x14, #0]
	fmla	v6.2d, v16.2d, v23.d[0]
	prfm	PLDL1KEEP, [x15, #0]
	fmla	v7.2d, v17.2d, v23.d[0]
	prfm	PLDL1KEEP, [x16, #16]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x17, #16]
//	fmla	v10.2d, v16.2d, v29.d[0]
//	fmla	v11.2d, v17.2d, v29.d[0]
//	prfm	PLDL1KEEP, [x18, #16]
//	fmla	v12.2d, v16.2d, v30.d[0]
//	fmla	v13.2d, v17.2d, v30.d[0]
//	prfm	PLDL1KEEP, [x19, #16]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 1
	ldp		q16, q17, [x9, #64]
	fmla	v0.2d, v18.2d, v20.d[1]
	fmla	v1.2d, v19.2d, v20.d[1]
//	ldr		q24, [x10], #16
	fmla	v2.2d, v18.2d, v21.d[1]
	prfm	PLDL1KEEP, [x9, #160]
	fmla	v3.2d, v19.2d, v21.d[1]
//	ldr		q25, [x13], #16
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v5.2d, v19.2d, v22.d[1]
//	ldr		q26, [x14], #16
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v7.2d, v19.2d, v23.d[1]
//	ldr		q27, [x15], #16
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
//	fmla	v10.2d, v18.2d, v29.d[1]
//	fmla	v11.2d, v19.2d, v29.d[1]
//	ldr		q29, [x17], #16
//	fmla	v12.2d, v18.2d, v30.d[1]
//	fmla	v13.2d, v19.2d, v30.d[1]
//	ldr		q30, [x18], #16
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]
//	ldr		q31, [x19], #16

	// unroll 2
	ldp		q18, q19, [x9, #96]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, #128
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, #64]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
//	fmla	v10.2d, v16.2d, v29.d[0]
//	fmla	v11.2d, v17.2d, v29.d[0]
//	fmla	v12.2d, v16.2d, v30.d[0]
//	fmla	v13.2d, v17.2d, v30.d[0]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 3
	ldp		q16, q17, [x9, #0]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldp		q20, q24, [x10], #32
	fmla	v2.2d, v18.2d, v25.d[1]
	prfm	PLDL1KEEP, [x9, #96]
	fmla	v3.2d, v19.2d, v25.d[1]
	ldp		q21, q25, [x13], #32
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	ldp		q22, q26, [x14], #32
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	ldp		q23, q27, [x15], #32
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
//	fmla	v10.2d, v18.2d, v29.d[1]
//	fmla	v11.2d, v19.2d, v29.d[1]
//	ldr		q29, [x17], #16
//	fmla	v12.2d, v18.2d, v30.d[1]
//	fmla	v13.2d, v19.2d, v30.d[1]
//	ldr		q30, [x18], #16
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]
//	ldr		q31, [x19], #16

	bgt		1b


	// reduce

//	sub		x9, x9, #32
//	sub		x10, x10, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldp		q18, q19, [x9, #32]
	fmla	v0.2d, v16.2d, v20.d[0]
	fmla	v1.2d, v17.2d, v20.d[0]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
	fmla	v3.2d, v17.2d, v21.d[0]
//	prfm	PLDL1KEEP, [x10, #32]
	fmla	v4.2d, v16.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x13, #32]
	fmla	v5.2d, v17.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x14, #32]
	fmla	v6.2d, v16.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x15, #32]
	fmla	v7.2d, v17.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x16, #32]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x17, #32]
//	fmla	v10.2d, v16.2d, v29.d[0]
//	fmla	v11.2d, v17.2d, v29.d[0]
//	prfm	PLDL1KEEP, [x18, #32]
//	fmla	v12.2d, v16.2d, v30.d[0]
//	fmla	v13.2d, v17.2d, v30.d[0]
//	prfm	PLDL1KEEP, [x19, #32]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 1
	ldp		q16, q17, [x9, #64]
	fmla	v0.2d, v18.2d, v20.d[1]
	fmla	v1.2d, v19.2d, v20.d[1]
//	ldr		q24, [x10], #16
	fmla	v2.2d, v18.2d, v21.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v21.d[1]
//	ldr		q25, [x13], #16
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v5.2d, v19.2d, v22.d[1]
//	ldr		q26, [x14], #16
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v7.2d, v19.2d, v23.d[1]
//	ldr		q27, [x15], #16
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
//	fmla	v10.2d, v18.2d, v29.d[1]
//	fmla	v11.2d, v19.2d, v29.d[1]
//	ldr		q29, [x17], #16
//	fmla	v12.2d, v18.2d, v30.d[1]
//	fmla	v13.2d, v19.2d, v30.d[1]
//	ldr		q30, [x18], #16
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]
//	ldr		q31, [x19], #16

	// unroll 2
	ldp		q18, q19, [x9, #96]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, #128
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
//	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
//	fmla	v10.2d, v16.2d, v29.d[0]
//	fmla	v11.2d, v17.2d, v29.d[0]
//	fmla	v12.2d, v16.2d, v30.d[0]
//	fmla	v13.2d, v17.2d, v30.d[0]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 3
//	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
//	ldr		q24, [x10], #16
	fmla	v2.2d, v18.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v25.d[1]
//	ldr		q25, [x13], #16
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
//	ldr		q26, [x14], #16
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
//	ldr		q27, [x15], #16
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
//	ldr		q28, [x16], #16
//	fmla	v10.2d, v18.2d, v29.d[1]
//	fmla	v11.2d, v19.2d, v29.d[1]
//	ldr		q29, [x17], #16
//	fmla	v12.2d, v18.2d, v30.d[1]
//	fmla	v13.2d, v19.2d, v30.d[1]
//	ldr		q30, [x18], #16
//	fmla	v14.2d, v18.2d, v31.d[1]
//	fmla	v15.2d, v19.2d, v31.d[1]
//	ldr		q31, [x19], #16

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x10, x10, #32
	sub		x13, x13, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x16, x16, #16
//	sub		x17, x17, #16
//	sub		x18, x18, #16
//	sub		x19, x19, #16

3: // clean1-up loop

	// load 0
	ldr		d24, [x10], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
//	ldr		d29, [x17], #8
//	ldr		d30, [x18], #8
//	ldr		d31, [x19], #8
	ldp		q16, q17, [x9]
	add		x9, x9, #32

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
//	fmla	v10.2d, v16.2d, v29.d[0]
//	fmla	v11.2d, v17.2d, v29.d[0]
//	fmla	v12.2d, v16.2d, v30.d[0]
//	fmla	v13.2d, v17.2d, v30.d[0]
//	fmla	v14.2d, v16.2d, v31.d[0]
//	fmla	v15.2d, v17.2d, v31.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_4x5_lib4c)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_4X8_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_4x8_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5
	add		x18, x16, x20 // 6
	add		x19, x17, x20 // 7

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q31, [x19], #16
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q31, [x19], #16
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q31, [x19], #16
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q31, [x19], #16
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
	ldr		d30, [x18], #8
	ldr		d31, [x19], #8
	ldp		q16, q17, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5
	add		x18, x16, x20 // 6
	add		x19, x17, x20 // 7

//	lsl		x20, x10, #2 // 4*lda*...
	lsl		x20, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x17, #0]
	prfm	PLDL1KEEP, [x18, #0]
	prfm	PLDL1KEEP, [x19, #0]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x20]

	// preload
	ldp		q20, q24, [x11], #32
	ldp		q21, q25, [x13], #32
	ldp		q22, q26, [x14], #32
	ldp		q23, q27, [x15], #32
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q31, [x19], #16
	ldp		q16, q17, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, x20]

	// zero tmp acc

	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v20.d[0]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
	fmla	v3.2d, v17.2d, v21.d[0]
	prfm	PLDL1KEEP, [x11, #0]
	fmla	v4.2d, v16.2d, v22.d[0]
	prfm	PLDL1KEEP, [x13, #0]
	fmla	v5.2d, v17.2d, v22.d[0]
	prfm	PLDL1KEEP, [x14, #0]
	fmla	v6.2d, v16.2d, v23.d[0]
	prfm	PLDL1KEEP, [x15, #0]
	fmla	v7.2d, v17.2d, v23.d[0]
	prfm	PLDL1KEEP, [x16, #16]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x17, #16]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	prfm	PLDL1KEEP, [x18, #16]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
	prfm	PLDL1KEEP, [x19, #16]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 1
	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v20.d[1]
	fmla	v2.2d, v18.2d, v21.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v21.d[1]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v5.2d, v19.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v7.2d, v19.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	ldr		q30, [x18], #16
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]
	ldr		q31, [x19], #16

	// unroll 2
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 3
	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v24.d[1]
	ldp		q20, q24, [x11], #32
	fmla	v2.2d, v18.2d, v25.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v25.d[1]
	ldp		q21, q25, [x13], #32
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	ldp		q22, q26, [x14], #32
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	ldp		q23, q27, [x15], #32
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	ldr		q30, [x18], #16
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]
	ldr		q31, [x19], #16

	bgt		1b


	// reduce


0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v20.d[0]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
	fmla	v3.2d, v17.2d, v21.d[0]
//	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x13, #32]
	fmla	v5.2d, v17.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x14, #32]
	fmla	v6.2d, v16.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x15, #32]
	fmla	v7.2d, v17.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x16, #32]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x17, #32]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
//	prfm	PLDL1KEEP, [x18, #32]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
//	prfm	PLDL1KEEP, [x19, #32]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 1
	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v20.d[1]
	fmla	v2.2d, v18.2d, v21.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v21.d[1]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v5.2d, v19.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v7.2d, v19.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	ldr		q30, [x18], #16
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]
	ldr		q31, [x19], #16

	// unroll 2
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
//	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 3
//	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
//	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v24.d[1]
//	ldr		q24, [x11], #16
	fmla	v2.2d, v18.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v25.d[1]
//	ldr		q25, [x13], #16
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
//	ldr		q26, [x14], #16
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
//	ldr		q27, [x15], #16
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
//	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
//	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
//	ldr		q30, [x18], #16
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]
//	ldr		q31, [x19], #16

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x13, x13, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x16, x16, #16
	sub		x17, x17, #16
	sub		x18, x18, #16
	sub		x19, x19, #16
	sub		x9, x9, x10

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
	ldr		d30, [x18], #8
	ldr		d31, [x19], #8
	ldp		q16, q17, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_4x8_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_3X8_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_3x8_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5
	add		x18, x16, x20 // 6
	add		x19, x17, x20 // 7

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q31, [x19], #16
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q31, [x19], #16
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q31, [x19], #16
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q31, [x19], #16
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
	ldr		d30, [x18], #8
	ldr		d31, [x19], #8
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5
	add		x18, x16, x20 // 6
	add		x19, x17, x20 // 7

//	lsl		x20, x10, #2 // 4*lda*...
	lsl		x20, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x17, #0]
	prfm	PLDL1KEEP, [x18, #0]
	prfm	PLDL1KEEP, [x19, #0]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x20]

	// preload
	ldp		q20, q24, [x11], #32
	ldp		q21, q25, [x13], #32
	ldp		q22, q26, [x14], #32
	ldp		q23, q27, [x15], #32
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q31, [x19], #16
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, x20]

	// zero tmp acc

	// main loop
1:
	
	// unroll 0
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v20.d[0]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
	fmla	v3.2d, v17.2d, v21.d[0]
	prfm	PLDL1KEEP, [x11, #0]
	fmla	v4.2d, v16.2d, v22.d[0]
	prfm	PLDL1KEEP, [x13, #0]
	fmla	v5.2d, v17.2d, v22.d[0]
	prfm	PLDL1KEEP, [x14, #0]
	fmla	v6.2d, v16.2d, v23.d[0]
	prfm	PLDL1KEEP, [x15, #0]
	fmla	v7.2d, v17.2d, v23.d[0]
	prfm	PLDL1KEEP, [x16, #16]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x17, #16]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	prfm	PLDL1KEEP, [x18, #16]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
	prfm	PLDL1KEEP, [x19, #16]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 1
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v20.d[1]
	fmla	v2.2d, v18.2d, v21.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v21.d[1]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v5.2d, v19.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v7.2d, v19.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	ldr		q30, [x18], #16
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]
	ldr		q31, [x19], #16

	// unroll 2
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 3
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v24.d[1]
	ldp		q20, q24, [x11], #32
	fmla	v2.2d, v18.2d, v25.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v25.d[1]
	ldp		q21, q25, [x13], #32
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	ldp		q22, q26, [x14], #32
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	ldp		q23, q27, [x15], #32
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	ldr		q30, [x18], #16
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]
	ldr		q31, [x19], #16

	bgt		1b


	// reduce


0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v20.d[0]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
	fmla	v3.2d, v17.2d, v21.d[0]
//	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x13, #32]
	fmla	v5.2d, v17.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x14, #32]
	fmla	v6.2d, v16.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x15, #32]
	fmla	v7.2d, v17.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x16, #32]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x17, #32]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
//	prfm	PLDL1KEEP, [x18, #32]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
//	prfm	PLDL1KEEP, [x19, #32]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 1
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v20.d[1]
	fmla	v2.2d, v18.2d, v21.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v21.d[1]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v5.2d, v19.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v7.2d, v19.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	ldr		q30, [x18], #16
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]
	ldr		q31, [x19], #16

	// unroll 2
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
//	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]

	// unroll 3
//	ldr		q16, [x9]
//	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v24.d[1]
//	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v24.d[1]
//	ldr		q24, [x11], #16
	fmla	v2.2d, v18.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v25.d[1]
//	ldr		q25, [x13], #16
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
//	ldr		q26, [x14], #16
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
//	ldr		q27, [x15], #16
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
//	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
//	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
//	ldr		q30, [x18], #16
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]
//	ldr		q31, [x19], #16

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x13, x13, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x16, x16, #16
	sub		x17, x17, #16
	sub		x18, x18, #16
	sub		x19, x19, #16
	sub		x9, x9, x10

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
	ldr		d30, [x18], #8
	ldr		d31, [x19], #8
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]
	fmla	v15.2d, v17.2d, v31.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_3x8_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_2X8_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_2x8_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5
	add		x18, x16, x20 // 6
	add		x19, x17, x20 // 7

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q31, [x19], #16
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v14.2d, v18.2d, v31.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q31, [x19], #16
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v14.2d, v18.2d, v31.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q31, [x19], #16
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v14.2d, v18.2d, v31.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q31, [x19], #16
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v14.2d, v18.2d, v31.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
	ldr		d30, [x18], #8
	ldr		d31, [x19], #8
	ldr		q16, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5
	add		x18, x16, x20 // 6
	add		x19, x17, x20 // 7

//	lsl		x20, x10, #2 // 4*lda*...
	lsl		x20, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x17, #0]
	prfm	PLDL1KEEP, [x18, #0]
	prfm	PLDL1KEEP, [x19, #0]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x20]

	// preload
	ldp		q20, q24, [x11], #32
	ldp		q21, q25, [x13], #32
	ldp		q22, q26, [x14], #32
	ldp		q23, q27, [x15], #32
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q31, [x19], #16
	ldr		q16, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, x20]

	// zero tmp acc

	// main loop
1:
	
	// unroll 0
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
	prfm	PLDL1KEEP, [x11, #0]
	fmla	v4.2d, v16.2d, v22.d[0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	fmla	v6.2d, v16.2d, v23.d[0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #16]
	fmla	v8.2d, v16.2d, v28.d[0]
	prfm	PLDL1KEEP, [x17, #16]
	fmla	v10.2d, v16.2d, v29.d[0]
	prfm	PLDL1KEEP, [x18, #16]
	fmla	v12.2d, v16.2d, v30.d[0]
	prfm	PLDL1KEEP, [x19, #16]
	fmla	v14.2d, v16.2d, v31.d[0]

	// unroll 1
	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v21.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	ldr		q30, [x18], #16
	fmla	v14.2d, v18.2d, v31.d[1]
	ldr		q31, [x19], #16

	// unroll 2
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]

	// unroll 3
	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	ldp		q20, q24, [x11], #32
	fmla	v2.2d, v18.2d, v25.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	ldp		q21, q25, [x13], #32
	fmla	v4.2d, v18.2d, v26.d[1]
	ldp		q22, q26, [x14], #32
	fmla	v6.2d, v18.2d, v27.d[1]
	ldp		q23, q27, [x15], #32
	fmla	v8.2d, v18.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	ldr		q30, [x18], #16
	fmla	v14.2d, v18.2d, v31.d[1]
	ldr		q31, [x19], #16

	bgt		1b


	// reduce


0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
//	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x13, #32]
//	prfm	PLDL1KEEP, [x14, #32]
	fmla	v6.2d, v16.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x15, #32]
//	prfm	PLDL1KEEP, [x16, #32]
	fmla	v8.2d, v16.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x17, #32]
	fmla	v10.2d, v16.2d, v29.d[0]
//	prfm	PLDL1KEEP, [x18, #32]
	fmla	v12.2d, v16.2d, v30.d[0]
//	prfm	PLDL1KEEP, [x19, #32]
	fmla	v14.2d, v16.2d, v31.d[0]

	// unroll 1
	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v21.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	ldr		q30, [x18], #16
	fmla	v14.2d, v18.2d, v31.d[1]
	ldr		q31, [x19], #16

	// unroll 2
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
//	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]

	// unroll 3
//	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
//	add		x9, x9, x10
//	ldr		q24, [x11], #16
	fmla	v2.2d, v18.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
//	ldr		q25, [x13], #16
	fmla	v4.2d, v18.2d, v26.d[1]
//	ldr		q26, [x14], #16
	fmla	v6.2d, v18.2d, v27.d[1]
//	ldr		q27, [x15], #16
	fmla	v8.2d, v18.2d, v28.d[1]
//	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
//	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
//	ldr		q30, [x18], #16
	fmla	v14.2d, v18.2d, v31.d[1]
//	ldr		q31, [x19], #16

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x13, x13, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x16, x16, #16
	sub		x17, x17, #16
	sub		x18, x18, #16
	sub		x19, x19, #16
	sub		x9, x9, x10

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
	ldr		d30, [x18], #8
	ldr		d31, [x19], #8
	ldr		q16, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_2x8_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_1X8_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_1x8_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5
	add		x18, x16, x20 // 6
	add		x19, x17, x20 // 7

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q31, [x19], #16
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v14.2d, v18.2d, v31.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q31, [x19], #16
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v14.2d, v18.2d, v31.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q31, [x19], #16
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v14.2d, v18.2d, v31.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q31, [x19], #16
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v14.2d, v18.2d, v31.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
	ldr		d30, [x18], #8
	ldr		d31, [x19], #8
	ldr		d16, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5
	add		x18, x16, x20 // 6
	add		x19, x17, x20 // 7

//	lsl		x20, x10, #2 // 4*lda*...
	lsl		x20, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x17, #0]
	prfm	PLDL1KEEP, [x18, #0]
	prfm	PLDL1KEEP, [x19, #0]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x20]

	// preload
	ldp		q20, q24, [x11], #32
	ldp		q21, q25, [x13], #32
	ldp		q22, q26, [x14], #32
	ldp		q23, q27, [x15], #32
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q31, [x19], #16
	ldr		d16, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, x20]

	// zero tmp acc

	// main loop
1:
	
	// unroll 0
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
	prfm	PLDL1KEEP, [x11, #0]
	fmla	v4.2d, v16.2d, v22.d[0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	fmla	v6.2d, v16.2d, v23.d[0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #16]
	fmla	v8.2d, v16.2d, v28.d[0]
	prfm	PLDL1KEEP, [x17, #16]
	fmla	v10.2d, v16.2d, v29.d[0]
	prfm	PLDL1KEEP, [x18, #16]
	fmla	v12.2d, v16.2d, v30.d[0]
	prfm	PLDL1KEEP, [x19, #16]
	fmla	v14.2d, v16.2d, v31.d[0]

	// unroll 1
	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v21.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	ldr		q30, [x18], #16
	fmla	v14.2d, v18.2d, v31.d[1]
	ldr		q31, [x19], #16

	// unroll 2
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]

	// unroll 3
	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	ldp		q20, q24, [x11], #32
	fmla	v2.2d, v18.2d, v25.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	ldp		q21, q25, [x13], #32
	fmla	v4.2d, v18.2d, v26.d[1]
	ldp		q22, q26, [x14], #32
	fmla	v6.2d, v18.2d, v27.d[1]
	ldp		q23, q27, [x15], #32
	fmla	v8.2d, v18.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	ldr		q30, [x18], #16
	fmla	v14.2d, v18.2d, v31.d[1]
	ldr		q31, [x19], #16

	bgt		1b


	// reduce


0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
//	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x13, #32]
//	prfm	PLDL1KEEP, [x14, #32]
	fmla	v6.2d, v16.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x15, #32]
//	prfm	PLDL1KEEP, [x16, #32]
	fmla	v8.2d, v16.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x17, #32]
	fmla	v10.2d, v16.2d, v29.d[0]
//	prfm	PLDL1KEEP, [x18, #32]
	fmla	v12.2d, v16.2d, v30.d[0]
//	prfm	PLDL1KEEP, [x19, #32]
	fmla	v14.2d, v16.2d, v31.d[0]

	// unroll 1
	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v21.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	ldr		q30, [x18], #16
	fmla	v14.2d, v18.2d, v31.d[1]
	ldr		q31, [x19], #16

	// unroll 2
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
//	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]

	// unroll 3
//	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
//	add		x9, x9, x10
//	ldr		q24, [x11], #16
	fmla	v2.2d, v18.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
//	ldr		q25, [x13], #16
	fmla	v4.2d, v18.2d, v26.d[1]
//	ldr		q26, [x14], #16
	fmla	v6.2d, v18.2d, v27.d[1]
//	ldr		q27, [x15], #16
	fmla	v8.2d, v18.2d, v28.d[1]
//	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
//	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
//	ldr		q30, [x18], #16
	fmla	v14.2d, v18.2d, v31.d[1]
//	ldr		q31, [x19], #16

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x13, x13, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x16, x16, #16
	sub		x17, x17, #16
	sub		x18, x18, #16
	sub		x19, x19, #16
	sub		x9, x9, x10

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
	ldr		d30, [x18], #8
	ldr		d31, [x19], #8
	ldr		d16, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v14.2d, v16.2d, v31.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_1x8_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_4X7_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_4x7_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5
	add		x18, x16, x20 // 6

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
	ldr		d30, [x18], #8
	ldp		q16, q17, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5
	add		x18, x16, x20 // 6

//	lsl		x20, x10, #2 // 4*lda*...
	lsl		x20, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x17, #0]
	prfm	PLDL1KEEP, [x18, #0]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x20]

	// preload
	ldp		q20, q24, [x11], #32
	ldp		q21, q25, [x13], #32
	ldp		q22, q26, [x14], #32
	ldp		q23, q27, [x15], #32
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldp		q16, q17, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, x20]

	// zero tmp acc

	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v20.d[0]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
	fmla	v3.2d, v17.2d, v21.d[0]
	prfm	PLDL1KEEP, [x11, #0]
	fmla	v4.2d, v16.2d, v22.d[0]
	prfm	PLDL1KEEP, [x13, #0]
	fmla	v5.2d, v17.2d, v22.d[0]
	prfm	PLDL1KEEP, [x14, #0]
	fmla	v6.2d, v16.2d, v23.d[0]
	prfm	PLDL1KEEP, [x15, #0]
	fmla	v7.2d, v17.2d, v23.d[0]
	prfm	PLDL1KEEP, [x16, #16]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x17, #16]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	prfm	PLDL1KEEP, [x18, #16]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]

	// unroll 1
	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v20.d[1]
	fmla	v2.2d, v18.2d, v21.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v21.d[1]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v5.2d, v19.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v7.2d, v19.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	ldr		q30, [x18], #16

	// unroll 2
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]

	// unroll 3
	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v24.d[1]
	ldp		q20, q24, [x11], #32
	fmla	v2.2d, v18.2d, v25.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v25.d[1]
	ldp		q21, q25, [x13], #32
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	ldp		q22, q26, [x14], #32
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	ldp		q23, q27, [x15], #32
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	ldr		q30, [x18], #16

	bgt		1b


	// reduce


0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v20.d[0]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
	fmla	v3.2d, v17.2d, v21.d[0]
//	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x13, #32]
	fmla	v5.2d, v17.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x14, #32]
	fmla	v6.2d, v16.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x15, #32]
	fmla	v7.2d, v17.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x16, #32]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x17, #32]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
//	prfm	PLDL1KEEP, [x18, #32]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]

	// unroll 1
	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v20.d[1]
	fmla	v2.2d, v18.2d, v21.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v21.d[1]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v5.2d, v19.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v7.2d, v19.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	ldr		q30, [x18], #16

	// unroll 2
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
//	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]

	// unroll 3
//	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
//	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v24.d[1]
//	ldr		q24, [x11], #16
	fmla	v2.2d, v18.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v25.d[1]
//	ldr		q25, [x13], #16
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
//	ldr		q26, [x14], #16
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
//	ldr		q27, [x15], #16
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
//	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
//	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
//	ldr		q30, [x18], #16

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x13, x13, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x16, x16, #16
	sub		x17, x17, #16
	sub		x18, x18, #16
	sub		x9, x9, x10

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
	ldr		d30, [x18], #8
	ldp		q16, q17, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_4x7_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_3X7_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_3x7_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5
	add		x18, x16, x20 // 6

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
	ldr		d30, [x18], #8
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5
	add		x18, x16, x20 // 6

//	lsl		x20, x10, #2 // 4*lda*...
	lsl		x20, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x17, #0]
	prfm	PLDL1KEEP, [x18, #0]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x20]

	// preload
	ldp		q20, q24, [x11], #32
	ldp		q21, q25, [x13], #32
	ldp		q22, q26, [x14], #32
	ldp		q23, q27, [x15], #32
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, x20]

	// zero tmp acc

	// main loop
1:
	
	// unroll 0
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v20.d[0]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
	fmla	v3.2d, v17.2d, v21.d[0]
	prfm	PLDL1KEEP, [x11, #0]
	fmla	v4.2d, v16.2d, v22.d[0]
	prfm	PLDL1KEEP, [x13, #0]
	fmla	v5.2d, v17.2d, v22.d[0]
	prfm	PLDL1KEEP, [x14, #0]
	fmla	v6.2d, v16.2d, v23.d[0]
	prfm	PLDL1KEEP, [x15, #0]
	fmla	v7.2d, v17.2d, v23.d[0]
	prfm	PLDL1KEEP, [x16, #16]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x17, #16]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	prfm	PLDL1KEEP, [x18, #16]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]

	// unroll 1
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v20.d[1]
	fmla	v2.2d, v18.2d, v21.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v21.d[1]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v5.2d, v19.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v7.2d, v19.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	ldr		q30, [x18], #16

	// unroll 2
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]

	// unroll 3
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v24.d[1]
	ldp		q20, q24, [x11], #32
	fmla	v2.2d, v18.2d, v25.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v25.d[1]
	ldp		q21, q25, [x13], #32
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	ldp		q22, q26, [x14], #32
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	ldp		q23, q27, [x15], #32
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	ldr		q30, [x18], #16

	bgt		1b


	// reduce


0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v20.d[0]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
	fmla	v3.2d, v17.2d, v21.d[0]
//	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x13, #32]
	fmla	v5.2d, v17.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x14, #32]
	fmla	v6.2d, v16.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x15, #32]
	fmla	v7.2d, v17.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x16, #32]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x17, #32]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
//	prfm	PLDL1KEEP, [x18, #32]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]

	// unroll 1
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v20.d[1]
	fmla	v2.2d, v18.2d, v21.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v21.d[1]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v5.2d, v19.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v7.2d, v19.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
	ldr		q30, [x18], #16

	// unroll 2
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
//	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]

	// unroll 3
//	ldr		q16, [x9]
//	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v24.d[1]
//	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v24.d[1]
//	ldr		q24, [x11], #16
	fmla	v2.2d, v18.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v25.d[1]
//	ldr		q25, [x13], #16
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
//	ldr		q26, [x14], #16
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
//	ldr		q27, [x15], #16
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
//	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
//	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	fmla	v13.2d, v19.2d, v30.d[1]
//	ldr		q30, [x18], #16

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x13, x13, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x16, x16, #16
	sub		x17, x17, #16
	sub		x18, x18, #16
	sub		x9, x9, x10

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
	ldr		d30, [x18], #8
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	fmla	v13.2d, v17.2d, v30.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_3x7_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_2X7_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_2x7_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5
	add		x18, x16, x20 // 6

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
	ldr		d30, [x18], #8
	ldr		q16, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5
	add		x18, x16, x20 // 6

//	lsl		x20, x10, #2 // 4*lda*...
	lsl		x20, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x17, #0]
	prfm	PLDL1KEEP, [x18, #0]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x20]

	// preload
	ldp		q20, q24, [x11], #32
	ldp		q21, q25, [x13], #32
	ldp		q22, q26, [x14], #32
	ldp		q23, q27, [x15], #32
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		q16, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, x20]

	// zero tmp acc

	// main loop
1:
	
	// unroll 0
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
	prfm	PLDL1KEEP, [x11, #0]
	fmla	v4.2d, v16.2d, v22.d[0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	fmla	v6.2d, v16.2d, v23.d[0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #16]
	fmla	v8.2d, v16.2d, v28.d[0]
	prfm	PLDL1KEEP, [x17, #16]
	fmla	v10.2d, v16.2d, v29.d[0]
	prfm	PLDL1KEEP, [x18, #16]
	fmla	v12.2d, v16.2d, v30.d[0]

	// unroll 1
	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v21.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	ldr		q30, [x18], #16

	// unroll 2
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]

	// unroll 3
	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	ldp		q20, q24, [x11], #32
	fmla	v2.2d, v18.2d, v25.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	ldp		q21, q25, [x13], #32
	fmla	v4.2d, v18.2d, v26.d[1]
	ldp		q22, q26, [x14], #32
	fmla	v6.2d, v18.2d, v27.d[1]
	ldp		q23, q27, [x15], #32
	fmla	v8.2d, v18.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	ldr		q30, [x18], #16

	bgt		1b


	// reduce


0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
//	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x13, #32]
//	prfm	PLDL1KEEP, [x14, #32]
	fmla	v6.2d, v16.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x15, #32]
//	prfm	PLDL1KEEP, [x16, #32]
	fmla	v8.2d, v16.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x17, #32]
	fmla	v10.2d, v16.2d, v29.d[0]
//	prfm	PLDL1KEEP, [x18, #32]
	fmla	v12.2d, v16.2d, v30.d[0]

	// unroll 1
	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v21.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	ldr		q30, [x18], #16

	// unroll 2
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
//	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]

	// unroll 3
//	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
//	add		x9, x9, x10
//	ldr		q24, [x11], #16
	fmla	v2.2d, v18.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
//	ldr		q25, [x13], #16
	fmla	v4.2d, v18.2d, v26.d[1]
//	ldr		q26, [x14], #16
	fmla	v6.2d, v18.2d, v27.d[1]
//	ldr		q27, [x15], #16
	fmla	v8.2d, v18.2d, v28.d[1]
//	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
//	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
//	ldr		q30, [x18], #16

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x13, x13, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x16, x16, #16
	sub		x17, x17, #16
	sub		x18, x18, #16
	sub		x9, x9, x10

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
	ldr		d30, [x18], #8
	ldr		q16, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_2x7_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_1X7_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_1x7_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5
	add		x18, x16, x20 // 6

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v12.2d, v18.2d, v30.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
	ldr		d30, [x18], #8
	ldr		d16, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5
	add		x18, x16, x20 // 6

//	lsl		x20, x10, #2 // 4*lda*...
	lsl		x20, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x17, #0]
	prfm	PLDL1KEEP, [x18, #0]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x20]

	// preload
	ldp		q20, q24, [x11], #32
	ldp		q21, q25, [x13], #32
	ldp		q22, q26, [x14], #32
	ldp		q23, q27, [x15], #32
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q30, [x18], #16
	ldr		d16, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, x20]

	// zero tmp acc

	// main loop
1:
	
	// unroll 0
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
	prfm	PLDL1KEEP, [x11, #0]
	fmla	v4.2d, v16.2d, v22.d[0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	fmla	v6.2d, v16.2d, v23.d[0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #16]
	fmla	v8.2d, v16.2d, v28.d[0]
	prfm	PLDL1KEEP, [x17, #16]
	fmla	v10.2d, v16.2d, v29.d[0]
	prfm	PLDL1KEEP, [x18, #16]
	fmla	v12.2d, v16.2d, v30.d[0]

	// unroll 1
	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v21.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	ldr		q30, [x18], #16

	// unroll 2
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]

	// unroll 3
	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	ldp		q20, q24, [x11], #32
	fmla	v2.2d, v18.2d, v25.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	ldp		q21, q25, [x13], #32
	fmla	v4.2d, v18.2d, v26.d[1]
	ldp		q22, q26, [x14], #32
	fmla	v6.2d, v18.2d, v27.d[1]
	ldp		q23, q27, [x15], #32
	fmla	v8.2d, v18.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	ldr		q30, [x18], #16

	bgt		1b


	// reduce


0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
//	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x13, #32]
//	prfm	PLDL1KEEP, [x14, #32]
	fmla	v6.2d, v16.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x15, #32]
//	prfm	PLDL1KEEP, [x16, #32]
	fmla	v8.2d, v16.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x17, #32]
	fmla	v10.2d, v16.2d, v29.d[0]
//	prfm	PLDL1KEEP, [x18, #32]
	fmla	v12.2d, v16.2d, v30.d[0]

	// unroll 1
	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v21.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
	ldr		q30, [x18], #16

	// unroll 2
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
//	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]

	// unroll 3
//	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
//	add		x9, x9, x10
//	ldr		q24, [x11], #16
	fmla	v2.2d, v18.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
//	ldr		q25, [x13], #16
	fmla	v4.2d, v18.2d, v26.d[1]
//	ldr		q26, [x14], #16
	fmla	v6.2d, v18.2d, v27.d[1]
//	ldr		q27, [x15], #16
	fmla	v8.2d, v18.2d, v28.d[1]
//	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
//	ldr		q29, [x17], #16
	fmla	v12.2d, v18.2d, v30.d[1]
//	ldr		q30, [x18], #16

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x13, x13, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x16, x16, #16
	sub		x17, x17, #16
	sub		x18, x18, #16
	sub		x9, x9, x10

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
	ldr		d30, [x18], #8
	ldr		d16, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v12.2d, v16.2d, v30.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_1x7_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_4X6_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_4x6_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
	ldp		q16, q17, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5

	lsl		x20, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x17, #0]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x20]

	// preload
	ldp		q20, q24, [x11], #32
	ldp		q21, q25, [x13], #32
	ldp		q22, q26, [x14], #32
	ldp		q23, q27, [x15], #32
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldp		q16, q17, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, x20]

	// zero tmp acc

	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v20.d[0]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
	fmla	v3.2d, v17.2d, v21.d[0]
	prfm	PLDL1KEEP, [x11, #0]
	fmla	v4.2d, v16.2d, v22.d[0]
	prfm	PLDL1KEEP, [x13, #0]
	fmla	v5.2d, v17.2d, v22.d[0]
	prfm	PLDL1KEEP, [x14, #0]
	fmla	v6.2d, v16.2d, v23.d[0]
	prfm	PLDL1KEEP, [x15, #0]
	fmla	v7.2d, v17.2d, v23.d[0]
	prfm	PLDL1KEEP, [x16, #16]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x17, #16]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]

	// unroll 1
	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v20.d[1]
	fmla	v2.2d, v18.2d, v21.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v21.d[1]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v5.2d, v19.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v7.2d, v19.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	ldr		q29, [x17], #16

	// unroll 2
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]

	// unroll 3
	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v24.d[1]
	ldp		q20, q24, [x11], #32
	fmla	v2.2d, v18.2d, v25.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v25.d[1]
	ldp		q21, q25, [x13], #32
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	ldp		q22, q26, [x14], #32
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	ldp		q23, q27, [x15], #32
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	ldr		q29, [x17], #16

	bgt		1b


	// reduce


0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v20.d[0]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
	fmla	v3.2d, v17.2d, v21.d[0]
//	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x13, #32]
	fmla	v5.2d, v17.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x14, #32]
	fmla	v6.2d, v16.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x15, #32]
	fmla	v7.2d, v17.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x16, #32]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x17, #32]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]

	// unroll 1
	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v20.d[1]
	fmla	v2.2d, v18.2d, v21.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v21.d[1]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v5.2d, v19.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v7.2d, v19.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	ldr		q29, [x17], #16

	// unroll 2
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
//	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]

	// unroll 3
//	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
//	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v24.d[1]
//	ldr		q24, [x11], #16
	fmla	v2.2d, v18.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v25.d[1]
//	ldr		q25, [x13], #16
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
//	ldr		q26, [x14], #16
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
//	ldr		q27, [x15], #16
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
//	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
//	ldr		q29, [x17], #16

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x13, x13, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x16, x16, #16
	sub		x17, x17, #16
	sub		x9, x9, x10

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
	ldp		q16, q17, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_4x6_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_3X6_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_3x6_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5

	lsl		x20, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x17, #0]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x20]

	// preload
	ldp		q20, q24, [x11], #32
	ldp		q21, q25, [x13], #32
	ldp		q22, q26, [x14], #32
	ldp		q23, q27, [x15], #32
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, x20]

	// zero tmp acc

	// main loop
1:
	
	// unroll 0
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v20.d[0]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
	fmla	v3.2d, v17.2d, v21.d[0]
	prfm	PLDL1KEEP, [x11, #0]
	fmla	v4.2d, v16.2d, v22.d[0]
	prfm	PLDL1KEEP, [x13, #0]
	fmla	v5.2d, v17.2d, v22.d[0]
	prfm	PLDL1KEEP, [x14, #0]
	fmla	v6.2d, v16.2d, v23.d[0]
	prfm	PLDL1KEEP, [x15, #0]
	fmla	v7.2d, v17.2d, v23.d[0]
	prfm	PLDL1KEEP, [x16, #16]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x17, #16]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]

	// unroll 1
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v20.d[1]
	fmla	v2.2d, v18.2d, v21.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v21.d[1]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v5.2d, v19.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v7.2d, v19.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	ldr		q29, [x17], #16

	// unroll 2
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]

	// unroll 3
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v24.d[1]
	ldp		q20, q24, [x11], #32
	fmla	v2.2d, v18.2d, v25.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v25.d[1]
	ldp		q21, q25, [x13], #32
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	ldp		q22, q26, [x14], #32
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	ldp		q23, q27, [x15], #32
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	ldr		q29, [x17], #16

	bgt		1b


	// reduce


0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v20.d[0]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
	fmla	v3.2d, v17.2d, v21.d[0]
//	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x13, #32]
	fmla	v5.2d, v17.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x14, #32]
	fmla	v6.2d, v16.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x15, #32]
	fmla	v7.2d, v17.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x16, #32]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x17, #32]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]

	// unroll 1
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v20.d[1]
	fmla	v2.2d, v18.2d, v21.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v21.d[1]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v5.2d, v19.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v7.2d, v19.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
	ldr		q29, [x17], #16

	// unroll 2
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
//	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]

	// unroll 3
//	ldr		q16, [x9]
//	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v24.d[1]
//	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v24.d[1]
//	ldr		q24, [x11], #16
	fmla	v2.2d, v18.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v25.d[1]
//	ldr		q25, [x13], #16
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
//	ldr		q26, [x14], #16
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
//	ldr		q27, [x15], #16
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
//	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	fmla	v11.2d, v19.2d, v29.d[1]
//	ldr		q29, [x17], #16

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x13, x13, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x16, x16, #16
	sub		x17, x17, #16
	sub		x9, x9, x10

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	fmla	v11.2d, v17.2d, v29.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_3x6_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_2X6_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_2x6_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
	ldr		q16, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5

	lsl		x20, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x17, #0]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x20]

	// preload
	ldp		q20, q24, [x11], #32
	ldp		q21, q25, [x13], #32
	ldp		q22, q26, [x14], #32
	ldp		q23, q27, [x15], #32
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		q16, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, x20]

	// zero tmp acc

	// main loop
1:
	
	// unroll 0
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
	prfm	PLDL1KEEP, [x11, #0]
	fmla	v4.2d, v16.2d, v22.d[0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	fmla	v6.2d, v16.2d, v23.d[0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #16]
	fmla	v8.2d, v16.2d, v28.d[0]
	prfm	PLDL1KEEP, [x17, #16]
	fmla	v10.2d, v16.2d, v29.d[0]

	// unroll 1
	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v21.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	ldr		q29, [x17], #16

	// unroll 2
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]

	// unroll 3
	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	ldp		q20, q24, [x11], #32
	fmla	v2.2d, v18.2d, v25.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	ldp		q21, q25, [x13], #32
	fmla	v4.2d, v18.2d, v26.d[1]
	ldp		q22, q26, [x14], #32
	fmla	v6.2d, v18.2d, v27.d[1]
	ldp		q23, q27, [x15], #32
	fmla	v8.2d, v18.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	ldr		q29, [x17], #16

	bgt		1b


	// reduce


0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
//	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x13, #32]
//	prfm	PLDL1KEEP, [x14, #32]
	fmla	v6.2d, v16.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x15, #32]
//	prfm	PLDL1KEEP, [x16, #32]
	fmla	v8.2d, v16.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x17, #32]
	fmla	v10.2d, v16.2d, v29.d[0]

	// unroll 1
	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v21.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	ldr		q29, [x17], #16

	// unroll 2
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
//	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]

	// unroll 3
//	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
//	add		x9, x9, x10
//	ldr		q24, [x11], #16
	fmla	v2.2d, v18.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
//	ldr		q25, [x13], #16
	fmla	v4.2d, v18.2d, v26.d[1]
//	ldr		q26, [x14], #16
	fmla	v6.2d, v18.2d, v27.d[1]
//	ldr		q27, [x15], #16
	fmla	v8.2d, v18.2d, v28.d[1]
//	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
//	ldr		q29, [x17], #16

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x13, x13, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x16, x16, #16
	sub		x17, x17, #16
	sub		x9, x9, x10

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
	ldr		q16, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_2x6_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_1X6_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_1x6_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v10.2d, v18.2d, v29.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
	ldr		d16, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4
	add		x17, x15, x20 // 5

	lsl		x20, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x17, #0]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x20]

	// preload
	ldp		q20, q24, [x11], #32
	ldp		q21, q25, [x13], #32
	ldp		q22, q26, [x14], #32
	ldp		q23, q27, [x15], #32
	ldr		q28, [x16], #16
	ldr		q29, [x17], #16
	ldr		d16, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, x20]

	// zero tmp acc

	// main loop
1:
	
	// unroll 0
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
	prfm	PLDL1KEEP, [x11, #0]
	fmla	v4.2d, v16.2d, v22.d[0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	fmla	v6.2d, v16.2d, v23.d[0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #16]
	fmla	v8.2d, v16.2d, v28.d[0]
	prfm	PLDL1KEEP, [x17, #16]
	fmla	v10.2d, v16.2d, v29.d[0]

	// unroll 1
	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v21.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	ldr		q29, [x17], #16

	// unroll 2
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]

	// unroll 3
	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	ldp		q20, q24, [x11], #32
	fmla	v2.2d, v18.2d, v25.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	ldp		q21, q25, [x13], #32
	fmla	v4.2d, v18.2d, v26.d[1]
	ldp		q22, q26, [x14], #32
	fmla	v6.2d, v18.2d, v27.d[1]
	ldp		q23, q27, [x15], #32
	fmla	v8.2d, v18.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	ldr		q29, [x17], #16

	bgt		1b


	// reduce


0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
//	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x13, #32]
//	prfm	PLDL1KEEP, [x14, #32]
	fmla	v6.2d, v16.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x15, #32]
//	prfm	PLDL1KEEP, [x16, #32]
	fmla	v8.2d, v16.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x17, #32]
	fmla	v10.2d, v16.2d, v29.d[0]

	// unroll 1
	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v21.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
	ldr		q29, [x17], #16

	// unroll 2
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
//	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]

	// unroll 3
//	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
//	add		x9, x9, x10
//	ldr		q24, [x11], #16
	fmla	v2.2d, v18.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
//	ldr		q25, [x13], #16
	fmla	v4.2d, v18.2d, v26.d[1]
//	ldr		q26, [x14], #16
	fmla	v6.2d, v18.2d, v27.d[1]
//	ldr		q27, [x15], #16
	fmla	v8.2d, v18.2d, v28.d[1]
//	ldr		q28, [x16], #16
	fmla	v10.2d, v18.2d, v29.d[1]
//	ldr		q29, [x17], #16

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x13, x13, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x16, x16, #16
	sub		x17, x17, #16
	sub		x9, x9, x10

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d29, [x17], #8
	ldr		d16, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v10.2d, v16.2d, v29.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_1x6_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_4X5_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_4x5_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldp		q16, q17, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4

//	lsl		x20, x10, #2 // 4*lda*...
	lsl		x20, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x20]

	// preload
	ldp		q20, q24, [x11], #32
	ldp		q21, q25, [x13], #32
	ldp		q22, q26, [x14], #32
	ldp		q23, q27, [x15], #32
	ldr		q28, [x16], #16
	ldp		q16, q17, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, x20]

	// zero tmp acc


	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v20.d[0]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
	fmla	v3.2d, v17.2d, v21.d[0]
	prfm	PLDL1KEEP, [x11, #0]
	fmla	v4.2d, v16.2d, v22.d[0]
	prfm	PLDL1KEEP, [x13, #0]
	fmla	v5.2d, v17.2d, v22.d[0]
	prfm	PLDL1KEEP, [x14, #0]
	fmla	v6.2d, v16.2d, v23.d[0]
	prfm	PLDL1KEEP, [x15, #0]
	fmla	v7.2d, v17.2d, v23.d[0]
	prfm	PLDL1KEEP, [x16, #16]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]

	// unroll 1
	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v20.d[1]
	fmla	v2.2d, v18.2d, v21.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v21.d[1]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v5.2d, v19.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v7.2d, v19.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16

	// unroll 2
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]

	// unroll 3
	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v24.d[1]
	ldp		q20, q24, [x11], #32
	fmla	v2.2d, v18.2d, v25.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v25.d[1]
	ldp		q21, q25, [x13], #32
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	ldp		q22, q26, [x14], #32
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	ldp		q23, q27, [x15], #32
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16

	bgt		1b


	// reduce

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v20.d[0]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
	fmla	v3.2d, v17.2d, v21.d[0]
//	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x13, #32]
	fmla	v5.2d, v17.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x14, #32]
	fmla	v6.2d, v16.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x15, #32]
	fmla	v7.2d, v17.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x16, #32]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]

	// unroll 1
	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v20.d[1]
	fmla	v2.2d, v18.2d, v21.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v21.d[1]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v5.2d, v19.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v7.2d, v19.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16

	// unroll 2
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
//	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]

	// unroll 3
//	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
//	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v24.d[1]
//	ldr		q24, [x11], #16
	fmla	v2.2d, v18.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v25.d[1]
//	ldr		q25, [x13], #16
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
//	ldr		q26, [x14], #16
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
//	ldr		q27, [x15], #16
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
//	ldr		q28, [x16], #16

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x13, x13, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x16, x16, #16
	sub		x9, x9, x10

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldp		q16, q17, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_4x5_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_3X5_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_3x5_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v3.2d, v19.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4

//	lsl		x20, x10, #2 // 4*lda*...
	lsl		x20, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x20]

	// preload
	ldp		q20, q24, [x11], #32
	ldp		q21, q25, [x13], #32
	ldp		q22, q26, [x14], #32
	ldp		q23, q27, [x15], #32
	ldr		q28, [x16], #16
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, x20]

	// zero tmp acc


	// main loop
1:
	
	// unroll 0
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v20.d[0]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
	fmla	v3.2d, v17.2d, v21.d[0]
	prfm	PLDL1KEEP, [x11, #0]
	fmla	v4.2d, v16.2d, v22.d[0]
	prfm	PLDL1KEEP, [x13, #0]
	fmla	v5.2d, v17.2d, v22.d[0]
	prfm	PLDL1KEEP, [x14, #0]
	fmla	v6.2d, v16.2d, v23.d[0]
	prfm	PLDL1KEEP, [x15, #0]
	fmla	v7.2d, v17.2d, v23.d[0]
	prfm	PLDL1KEEP, [x16, #16]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]

	// unroll 1
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v20.d[1]
	fmla	v2.2d, v18.2d, v21.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v21.d[1]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v5.2d, v19.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v7.2d, v19.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16

	// unroll 2
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]

	// unroll 3
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v24.d[1]
	ldp		q20, q24, [x11], #32
	fmla	v2.2d, v18.2d, v25.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v25.d[1]
	ldp		q21, q25, [x13], #32
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
	ldp		q22, q26, [x14], #32
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	ldp		q23, q27, [x15], #32
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16

	bgt		1b


	// reduce

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v20.d[0]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
	fmla	v3.2d, v17.2d, v21.d[0]
//	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x13, #32]
	fmla	v5.2d, v17.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x14, #32]
	fmla	v6.2d, v16.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x15, #32]
	fmla	v7.2d, v17.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x16, #32]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]

	// unroll 1
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v20.d[1]
	fmla	v2.2d, v18.2d, v21.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v21.d[1]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v5.2d, v19.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v7.2d, v19.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
	ldr		q28, [x16], #16

	// unroll 2
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
//	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]

	// unroll 3
//	ldr		q16, [x9]
//	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v24.d[1]
//	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v24.d[1]
//	ldr		q24, [x11], #16
	fmla	v2.2d, v18.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v3.2d, v19.2d, v25.d[1]
//	ldr		q25, [x13], #16
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v5.2d, v19.2d, v26.d[1]
//	ldr		q26, [x14], #16
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
//	ldr		q27, [x15], #16
	fmla	v8.2d, v18.2d, v28.d[1]
	fmla	v9.2d, v19.2d, v28.d[1]
//	ldr		q28, [x16], #16

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x13, x13, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x16, x16, #16
	sub		x9, x9, x10

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v3.2d, v17.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v5.2d, v17.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v7.2d, v17.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	fmla	v9.2d, v17.2d, v28.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_3x5_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_2X5_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_2x5_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		q16, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4

//	lsl		x20, x10, #2 // 4*lda*...
	lsl		x20, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x20]

	// preload
	ldp		q20, q24, [x11], #32
	ldp		q21, q25, [x13], #32
	ldp		q22, q26, [x14], #32
	ldp		q23, q27, [x15], #32
	ldr		q28, [x16], #16
	ldr		q16, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, x20]

	// zero tmp acc


	// main loop
1:
	
	// unroll 0
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
	prfm	PLDL1KEEP, [x11, #0]
	fmla	v4.2d, v16.2d, v22.d[0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	fmla	v6.2d, v16.2d, v23.d[0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #16]
	fmla	v8.2d, v16.2d, v28.d[0]

	// unroll 1
	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v21.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	ldr		q28, [x16], #16

	// unroll 2
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]

	// unroll 3
	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	ldp		q20, q24, [x11], #32
	fmla	v2.2d, v18.2d, v25.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	ldp		q21, q25, [x13], #32
	fmla	v4.2d, v18.2d, v26.d[1]
	ldp		q22, q26, [x14], #32
	fmla	v6.2d, v18.2d, v27.d[1]
	ldp		q23, q27, [x15], #32
	fmla	v8.2d, v18.2d, v28.d[1]
	ldr		q28, [x16], #16

	bgt		1b


	// reduce

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
//	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x13, #32]
//	prfm	PLDL1KEEP, [x14, #32]
	fmla	v6.2d, v16.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x15, #32]
//	prfm	PLDL1KEEP, [x16, #32]
	fmla	v8.2d, v16.2d, v28.d[0]

	// unroll 1
	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v21.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	ldr		q28, [x16], #16

	// unroll 2
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
//	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]

	// unroll 3
//	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
//	add		x9, x9, x10
//	ldr		q24, [x11], #16
	fmla	v2.2d, v18.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
//	ldr		q25, [x13], #16
	fmla	v4.2d, v18.2d, v26.d[1]
//	ldr		q26, [x14], #16
	fmla	v6.2d, v18.2d, v27.d[1]
//	ldr		q27, [x15], #16
	fmla	v8.2d, v18.2d, v28.d[1]
//	ldr		q28, [x16], #16

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x13, x13, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x16, x16, #16
	sub		x9, x9, x10

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		q16, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_2x5_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_1X5_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_1x5_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	sub		w8, w8, #4

	// load 2 & 3
	ldr		q24, [x11], #16
	ldr		q25, [x13], #16
	ldr		q26, [x14], #16
	ldr		q27, [x15], #16
	ldr		q28, [x16], #16
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10

	// unroll 2
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v25.d[1]
	fmla	v4.2d, v18.2d, v26.d[1]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d16, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x12, x12

	add		x13, x11, x12 // 1
	add		x14, x11, x20 // 2
	add		x15, x13, x20 // 3
	add		x16, x14, x20 // 4

//	lsl		x20, x10, #2 // 4*lda*...
	lsl		x20, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x20]

	// preload
	ldp		q20, q24, [x11], #32
	ldp		q21, q25, [x13], #32
	ldp		q22, q26, [x14], #32
	ldp		q23, q27, [x15], #32
	ldr		q28, [x16], #16
	ldr		d16, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, x20]

	// zero tmp acc


	// main loop
1:
	
	// unroll 0
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
	prfm	PLDL1KEEP, [x11, #0]
	fmla	v4.2d, v16.2d, v22.d[0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	fmla	v6.2d, v16.2d, v23.d[0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #16]
	fmla	v8.2d, v16.2d, v28.d[0]

	// unroll 1
	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v21.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	ldr		q28, [x16], #16

	// unroll 2
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]

	// unroll 3
	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	ldp		q20, q24, [x11], #32
	fmla	v2.2d, v18.2d, v25.d[1]
	prfm	PLDL1KEEP, [x9, x20]
	ldp		q21, q25, [x13], #32
	fmla	v4.2d, v18.2d, v26.d[1]
	ldp		q22, q26, [x14], #32
	fmla	v6.2d, v18.2d, v27.d[1]
	ldp		q23, q27, [x15], #32
	fmla	v8.2d, v18.2d, v28.d[1]
	ldr		q28, [x16], #16

	bgt		1b


	// reduce

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v20.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v21.d[0]
//	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v22.d[0]
//	prfm	PLDL1KEEP, [x13, #32]
//	prfm	PLDL1KEEP, [x14, #32]
	fmla	v6.2d, v16.2d, v23.d[0]
//	prfm	PLDL1KEEP, [x15, #32]
//	prfm	PLDL1KEEP, [x16, #32]
	fmla	v8.2d, v16.2d, v28.d[0]

	// unroll 1
	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v20.d[1]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v21.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v4.2d, v18.2d, v22.d[1]
	fmla	v6.2d, v18.2d, v23.d[1]
	fmla	v8.2d, v18.2d, v28.d[1]
	ldr		q28, [x16], #16

	// unroll 2
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, x20]
	fmla	v2.2d, v16.2d, v25.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v26.d[0]
//	cmp		w8, #4
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]

	// unroll 3
//	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
//	add		x9, x9, x10
//	ldr		q24, [x11], #16
	fmla	v2.2d, v18.2d, v25.d[1]
//	prfm	PLDL1KEEP, [x9, x20]
//	ldr		q25, [x13], #16
	fmla	v4.2d, v18.2d, v26.d[1]
//	ldr		q26, [x14], #16
	fmla	v6.2d, v18.2d, v27.d[1]
//	ldr		q27, [x15], #16
	fmla	v8.2d, v18.2d, v28.d[1]
//	ldr		q28, [x16], #16

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x13, x13, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x16, x16, #16
	sub		x9, x9, x10

3: // clean1-up loop

	// load 0
	ldr		d24, [x11], #8
	ldr		d25, [x13], #8
	ldr		d26, [x14], #8
	ldr		d27, [x15], #8
	ldr		d28, [x16], #8
	ldr		d16, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v25.d[0]
	fmla	v4.2d, v16.2d, v26.d[0]
	fmla	v6.2d, v16.2d, v27.d[0]
	fmla	v8.2d, v16.2d, v28.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_1x5_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_4X8_VS_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_4x8_vs_libcc)

	// save old return address
	sub sp, sp, #16
	str x30, [sp, #0]
#endif

	cmp		w13, #1
	bgt		90f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_1X8_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_1x8_libcc)
#endif

	b		93f

90:

	cmp		w13, #2
	bgt		91f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_2X8_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_2x8_libcc)
#endif
	
	b		93f

91:

	cmp		w13, #3
	bgt		92f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_3X8_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_3x8_libcc)
#endif
	
	b		93f

92:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X8_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_4x8_libcc)
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

	FUN_END(inner_kernel_gemm_add_nn_4x8_vs_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_4X7_VS_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_4x7_vs_libcc)

	// save old return address
	sub sp, sp, #16
	str x30, [sp, #0]
#endif

	cmp		w13, #1
	bgt		90f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_1X7_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_1x7_libcc)
#endif

	b		93f

90:

	cmp		w13, #2
	bgt		91f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_2X7_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_2x7_libcc)
#endif
	
	b		93f

91:

	cmp		w13, #3
	bgt		92f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_3X7_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_3x7_libcc)
#endif
	
	b		93f

92:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X7_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_4x7_libcc)
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

	FUN_END(inner_kernel_gemm_add_nn_4x7_vs_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_4X6_VS_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_4x6_vs_libcc)

	// save old return address
	sub sp, sp, #16
	str x30, [sp, #0]
#endif

	cmp		w13, #1
	bgt		90f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_1X6_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_1x6_libcc)
#endif

	b		93f

90:

	cmp		w13, #2
	bgt		91f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_2X6_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_2x6_libcc)
#endif
	
	b		93f

91:

	cmp		w13, #3
	bgt		92f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_3X6_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_3x6_libcc)
#endif
	
	b		93f

92:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X6_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_4x6_libcc)
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

	FUN_END(inner_kernel_gemm_add_nn_4x6_vs_libcc)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_4X5_VS_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_4x5_vs_libcc)

	// save old return address
	sub sp, sp, #16
	str x30, [sp, #0]
#endif

	cmp		w13, #1
	bgt		90f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_1X5_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_1x5_libcc)
#endif

	b		93f

90:

	cmp		w13, #2
	bgt		91f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_2X5_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_2x5_libcc)
#endif
	
	b		93f

91:

	cmp		w13, #3
	bgt		92f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_3X5_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_3x5_libcc)
#endif
	
	b		93f

92:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X5_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_4x5_libcc)
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

	FUN_END(inner_kernel_gemm_add_nn_4x5_vs_libcc)
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
// x10   <- sda
// x11   <- B
// x12   <- ldb
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRMM_NN_RL_8X4_LIB4C
#else
	.align 4
	FUN_START(inner_edge_trmm_nn_rl_8x4_lib4c)
#endif

	add		x13, x11, x12
	add		x14, x13, x12
	add		x15, x14, x12

	add		x16, x9, x10

	ldp		q24, q25, [x9, #(0*8+0*32)] // A
	ldr		d26, [x11, #(0*8)] // B
	ldp		q28, q29, [x16, #(0*8+0*32)] // A
	fmla	v0.2d, v24.2d, v26.d[0]
	fmla	v1.2d, v25.2d, v26.d[0]
	fmla	v8.2d, v28.2d, v26.d[0]
	fmla	v9.2d, v29.2d, v26.d[0]

	ldp		q24, q25, [x9, #(0*8+1*32)] // A
	ldr		d26, [x11, #(1*8)] // B
	ldp		q28, q29, [x16, #(0*8+1*32)] // A
	fmla	v0.2d, v24.2d, v26.d[0]
	fmla	v1.2d, v25.2d, v26.d[0]
	fmla	v8.2d, v28.2d, v26.d[0]
	fmla	v9.2d, v29.2d, v26.d[0]
	ldr		d26, [x13, #(1*8)] // B
	fmla	v2.2d, v24.2d, v26.d[0]
	fmla	v3.2d, v25.2d, v26.d[0]
	fmla	v10.2d, v28.2d, v26.d[0]
	fmla	v11.2d, v29.2d, v26.d[0]

	ldp		q24, q25, [x9, #(0*8+2*32)] // A
	ldr		d26, [x11, #(2*8)] // B
	ldp		q28, q29, [x16, #(0*8+2*32)] // A
	fmla	v0.2d, v24.2d, v26.d[0]
	fmla	v1.2d, v25.2d, v26.d[0]
	fmla	v8.2d, v28.2d, v26.d[0]
	fmla	v9.2d, v29.2d, v26.d[0]
	ldr		d26, [x13, #(2*8)] // B
	fmla	v2.2d, v24.2d, v26.d[0]
	fmla	v3.2d, v25.2d, v26.d[0]
	fmla	v10.2d, v28.2d, v26.d[0]
	fmla	v11.2d, v29.2d, v26.d[0]
	ldr		d26, [x14, #(2*8)] // B
	fmla	v4.2d, v24.2d, v26.d[0]
	fmla	v5.2d, v25.2d, v26.d[0]
	fmla	v12.2d, v28.2d, v26.d[0]
	fmla	v13.2d, v29.2d, v26.d[0]

	ldp		q24, q25, [x9, #(0*8+3*32)] // A
	ldr		d26, [x11, #(3*8)] // B
	ldp		q28, q29, [x16, #(0*8+3*32)] // A
	fmla	v0.2d, v24.2d, v26.d[0]
	fmla	v1.2d, v25.2d, v26.d[0]
	fmla	v8.2d, v28.2d, v26.d[0]
	fmla	v9.2d, v29.2d, v26.d[0]
	ldr		d26, [x13, #(3*8)] // B
	fmla	v2.2d, v24.2d, v26.d[0]
	fmla	v3.2d, v25.2d, v26.d[0]
	fmla	v10.2d, v28.2d, v26.d[0]
	fmla	v11.2d, v29.2d, v26.d[0]
	ldr		d26, [x14, #(3*8)] // B
	fmla	v4.2d, v24.2d, v26.d[0]
	fmla	v5.2d, v25.2d, v26.d[0]
	fmla	v12.2d, v28.2d, v26.d[0]
	fmla	v13.2d, v29.2d, v26.d[0]
	ldr		d26, [x15, #(3*8)] // B
	fmla	v6.2d, v24.2d, v26.d[0]
	fmla	v7.2d, v25.2d, v26.d[0]
	fmla	v14.2d, v28.2d, v26.d[0]
	fmla	v15.2d, v29.2d, v26.d[0]

	sub		w8, w8, #4
	add		x9, x9, #128
	add		x11, x11, #32

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trmm_nn_rl_8x4_lib4c)
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
// x10   <- sda
// x11   <- B
// x12   <- ldb
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRMM_NN_RL_8X4_VS_LIB4C
#else
	.align 4
	FUN_START(inner_edge_trmm_nn_rl_8x4_vs_lib4c)
#endif

	add		x13, x11, x12
	add		x14, x13, x12
	add		x15, x14, x12

	add		x16, x9, x10

	ldp		q24, q25, [x9, #(0*8+0*32)] // A
	ldr		d26, [x11, #(0*8)] // B
	ldp		q28, q29, [x16, #(0*8+0*32)] // A
	fmla	v0.2d, v24.2d, v26.d[0]
	fmla	v1.2d, v25.2d, v26.d[0]
	fmla	v8.2d, v28.2d, v26.d[0]
	fmla	v9.2d, v29.2d, v26.d[0]
	sub		w8, w8, #1
	add		x9, x9, #32
	add		x11, x11, #8
//	add		x13, x13, #8
//	add		x14, x14, #8
//	add		x15, x15, #8

	cmp		w8, #0
	ble		0f

	ldp		q24, q25, [x9, #(0*8+0*32)] // A
	ldr		d26, [x11, #(0*8)] // B
	ldp		q28, q29, [x16, #(0*8+1*32)] // A
	fmla	v0.2d, v24.2d, v26.d[0]
	fmla	v1.2d, v25.2d, v26.d[0]
	fmla	v8.2d, v28.2d, v26.d[0]
	fmla	v9.2d, v29.2d, v26.d[0]
	ldr		d26, [x13, #(1*8)] // B XXX
	fmla	v2.2d, v24.2d, v26.d[0]
	fmla	v3.2d, v25.2d, v26.d[0]
	fmla	v10.2d, v28.2d, v26.d[0]
	fmla	v11.2d, v29.2d, v26.d[0]
	sub		w8, w8, #1
	add		x9, x9, #32
	add		x11, x11, #8
//	add		x13, x13, #8
//	add		x14, x14, #8
//	add		x15, x15, #8

	cmp		w8, #0
	ble		0f

	ldp		q24, q25, [x9, #(0*8+0*32)] // A
	ldr		d26, [x11, #(0*8)] // B
	ldp		q28, q29, [x16, #(0*8+2*32)] // A
	fmla	v0.2d, v24.2d, v26.d[0]
	fmla	v1.2d, v25.2d, v26.d[0]
	fmla	v8.2d, v28.2d, v26.d[0]
	fmla	v9.2d, v29.2d, v26.d[0]
	ldr		d26, [x13, #(2*8)] // B XXX
	fmla	v2.2d, v24.2d, v26.d[0]
	fmla	v3.2d, v25.2d, v26.d[0]
	fmla	v10.2d, v28.2d, v26.d[0]
	fmla	v11.2d, v29.2d, v26.d[0]
	ldr		d26, [x14, #(2*8)] // B XXX
	fmla	v4.2d, v24.2d, v26.d[0]
	fmla	v5.2d, v25.2d, v26.d[0]
	fmla	v12.2d, v28.2d, v26.d[0]
	fmla	v13.2d, v29.2d, v26.d[0]
	sub		w8, w8, #1
	add		x9, x9, #32
	add		x11, x11, #8
//	add		x13, x13, #8
//	add		x14, x14, #8
//	add		x15, x15, #8

	cmp		w8, #0
	ble		0f

	ldp		q24, q25, [x9, #(0*8+0*32)] // A
	ldr		d26, [x11, #(0*8)] // B
	ldp		q28, q29, [x16, #(0*8+3*32)] // A
	fmla	v0.2d, v24.2d, v26.d[0]
	fmla	v1.2d, v25.2d, v26.d[0]
	fmla	v8.2d, v28.2d, v26.d[0]
	fmla	v9.2d, v29.2d, v26.d[0]
	ldr		d26, [x13, #(3*8)] // B XXX
	fmla	v2.2d, v24.2d, v26.d[0]
	fmla	v3.2d, v25.2d, v26.d[0]
	fmla	v10.2d, v28.2d, v26.d[0]
	fmla	v11.2d, v29.2d, v26.d[0]
	ldr		d26, [x14, #(3*8)] // B XXX
	fmla	v4.2d, v24.2d, v26.d[0]
	fmla	v5.2d, v25.2d, v26.d[0]
	fmla	v12.2d, v28.2d, v26.d[0]
	fmla	v13.2d, v29.2d, v26.d[0]
	ldr		d26, [x15, #(3*8)] // B XXX
	fmla	v6.2d, v24.2d, v26.d[0]
	fmla	v7.2d, v25.2d, v26.d[0]
	fmla	v14.2d, v28.2d, v26.d[0]
	fmla	v15.2d, v29.2d, v26.d[0]
	sub		w8, w8, #1
	add		x9, x9, #32
	add		x11, x11, #8
//	add		x13, x13, #8
//	add		x14, x14, #8
//	add		x15, x15, #8

	cmp		w8, #0
	ble		0f

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trmm_nn_rl_8x4_vs_lib4c)
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
// x10  <- sda
// x11  <- B
// w12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRMM_NT_RL_8X4_LIB4C
#else
	.align 4
	FUN_START(inner_edge_trmm_nt_rl_8x4_lib4c)
#endif

	add		x13, x9, x10

	ldp		q24, q25, [x9, #(0*8+0*32)] // A
	ldp		q26, q27, [x11, #(0*8)] // B
	fmla	v0.2d, v24.2d, v26.d[0]
	fmla	v1.2d, v25.2d, v26.d[0]
	fmla	v2.2d, v24.2d, v26.d[1]
	fmla	v3.2d, v25.2d, v26.d[1]
	fmla	v4.2d, v24.2d, v27.d[0]
	fmla	v5.2d, v25.2d, v27.d[0]
	fmla	v6.2d, v24.2d, v27.d[1]
	fmla	v7.2d, v25.2d, v27.d[1]
	ldp		q28, q29, [x13, #(0*8+0*32)] // A
	fmla	v8.2d, v28.2d, v26.d[0]
	fmla	v9.2d, v29.2d, v26.d[0]
	fmla	v10.2d, v28.2d, v26.d[1]
	fmla	v11.2d, v29.2d, v26.d[1]
	fmla	v12.2d, v28.2d, v27.d[0]
	fmla	v13.2d, v29.2d, v27.d[0]
	fmla	v14.2d, v28.2d, v27.d[1]
	fmla	v15.2d, v29.2d, v27.d[1]
	add		x11, x11, x12

	ldp		q24, q25, [x9, #(0*8+1*32)] // A
	ldr		d26, [x11, #(1*8)] // B
	ldr		q27, [x11, #(2*8)] // B
	fmla	v2.2d, v24.2d, v26.d[0]
	fmla	v3.2d, v25.2d, v26.d[0]
	fmla	v4.2d, v24.2d, v27.d[0]
	fmla	v5.2d, v25.2d, v27.d[0]
	fmla	v6.2d, v24.2d, v27.d[1]
	fmla	v7.2d, v25.2d, v27.d[1]
	ldp		q28, q29, [x13, #(0*8+1*32)] // A
	fmla	v10.2d, v28.2d, v26.d[0]
	fmla	v11.2d, v29.2d, v26.d[0]
	fmla	v12.2d, v28.2d, v27.d[0]
	fmla	v13.2d, v29.2d, v27.d[0]
	fmla	v14.2d, v28.2d, v27.d[1]
	fmla	v15.2d, v29.2d, v27.d[1]
	add		x11, x11, x12

	ldp		q24, q25, [x9, #(0*8+2*32)] // A
	ldr		q27, [x11, #(2*8)] // B
	fmla	v4.2d, v24.2d, v27.d[0]
	fmla	v5.2d, v25.2d, v27.d[0]
	fmla	v6.2d, v24.2d, v27.d[1]
	fmla	v7.2d, v25.2d, v27.d[1]
	ldp		q28, q29, [x13, #(0*8+2*32)] // A
	fmla	v12.2d, v28.2d, v27.d[0]
	fmla	v13.2d, v29.2d, v27.d[0]
	fmla	v14.2d, v28.2d, v27.d[1]
	fmla	v15.2d, v29.2d, v27.d[1]
	add		x11, x11, x12

	ldp		q24, q25, [x9, #(0*8+3*32)] // A
	ldr		d27, [x11, #(3*8)] // B
	fmla	v6.2d, v24.2d, v27.d[0]
	fmla	v7.2d, v25.2d, v27.d[0]
	ldp		q28, q29, [x13, #(0*8+3*32)] // A
	fmla	v14.2d, v28.2d, v27.d[0]
	fmla	v15.2d, v29.2d, v27.d[0]
	add		x11, x11, x12

	sub		w8, w8, #4
	add		x9, x9, #128

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trmm_nt_rl_8x4_lib4c)
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
// x10  <- sda
// x11  <- B
// w12  <- ldb
// w13  <- n1
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRMM_NT_RL_8X4_VS_LIB4C
#else
	.align 4
	FUN_START(inner_edge_trmm_nt_rl_8x4_vs_lib4c)
#endif

	cmp		w13, #0
	ble		0f

	add		x14, x9, x10

	cmp		w13, #4
	blt		1f

	ldp		q24, q25, [x9, #(0*8+0*32)] // A
	ldp		q26, q27, [x11, #(0*8)] // B
	fmla	v0.2d, v24.2d, v26.d[0]
	fmla	v1.2d, v25.2d, v26.d[0]
	fmla	v2.2d, v24.2d, v26.d[1]
	fmla	v3.2d, v25.2d, v26.d[1]
	fmla	v4.2d, v24.2d, v27.d[0]
	fmla	v5.2d, v25.2d, v27.d[0]
	fmla	v6.2d, v24.2d, v27.d[1]
	fmla	v7.2d, v25.2d, v27.d[1]
	ldp		q28, q29, [x14, #(0*8+0*32)] // A
	fmla	v8.2d, v28.2d, v26.d[0]
	fmla	v9.2d, v29.2d, v26.d[0]
	fmla	v10.2d, v28.2d, v26.d[1]
	fmla	v11.2d, v29.2d, v26.d[1]
	fmla	v12.2d, v28.2d, v27.d[0]
	fmla	v13.2d, v29.2d, v27.d[0]
	fmla	v14.2d, v28.2d, v27.d[1]
	fmla	v15.2d, v29.2d, v27.d[1]
	add		x11, x11, x12

	ldp		q24, q25, [x9, #(0*8+1*32)] // A
	ldr		d26, [x11, #(1*8)] // B
	ldr		q27, [x11, #(2*8)] // B
	fmla	v2.2d, v24.2d, v26.d[0]
	fmla	v3.2d, v25.2d, v26.d[0]
	fmla	v4.2d, v24.2d, v27.d[0]
	fmla	v5.2d, v25.2d, v27.d[0]
	fmla	v6.2d, v24.2d, v27.d[1]
	fmla	v7.2d, v25.2d, v27.d[1]
	ldp		q28, q29, [x14, #(0*8+1*32)] // A
	fmla	v10.2d, v28.2d, v26.d[0]
	fmla	v11.2d, v29.2d, v26.d[0]
	fmla	v12.2d, v28.2d, v27.d[0]
	fmla	v13.2d, v29.2d, v27.d[0]
	fmla	v14.2d, v28.2d, v27.d[1]
	fmla	v15.2d, v29.2d, v27.d[1]
	add		x11, x11, x12

	ldp		q24, q25, [x9, #(0*8+2*32)] // A
	ldr		q27, [x11, #(2*8)] // B
	fmla	v4.2d, v24.2d, v27.d[0]
	fmla	v5.2d, v25.2d, v27.d[0]
	fmla	v6.2d, v24.2d, v27.d[1]
	fmla	v7.2d, v25.2d, v27.d[1]
	ldp		q28, q29, [x14, #(0*8+2*32)] // A
	fmla	v12.2d, v28.2d, v27.d[0]
	fmla	v13.2d, v29.2d, v27.d[0]
	fmla	v14.2d, v28.2d, v27.d[1]
	fmla	v15.2d, v29.2d, v27.d[1]
	add		x11, x11, x12

	ldp		q24, q25, [x9, #(0*8+3*32)] // A
	ldr		d27, [x11, #(3*8)] // B
	fmla	v6.2d, v24.2d, v27.d[0]
	fmla	v7.2d, v25.2d, v27.d[0]
	ldp		q28, q29, [x14, #(0*8+3*32)] // A
	fmla	v14.2d, v28.2d, v27.d[0]
	fmla	v15.2d, v29.2d, v27.d[0]
	add		x11, x11, x12

	sub		w8, w8, #4
	add		x9, x9, #128

	b		0f

1:

	cmp		w13, #3
	blt		1f

	ldp		q24, q25, [x9, #(0*8+0*32)] // A
	ldr		q26, [x11, #(0*8)] // B
	ldr		d27, [x11, #(2*8)] // B
	fmla	v0.2d, v24.2d, v26.d[0]
	fmla	v1.2d, v25.2d, v26.d[0]
	fmla	v2.2d, v24.2d, v26.d[1]
	fmla	v3.2d, v25.2d, v26.d[1]
	fmla	v4.2d, v24.2d, v27.d[0]
	fmla	v5.2d, v25.2d, v27.d[0]
	ldp		q28, q29, [x14, #(0*8+0*32)] // A
	fmla	v8.2d, v28.2d, v26.d[0]
	fmla	v9.2d, v29.2d, v26.d[0]
	fmla	v10.2d, v28.2d, v26.d[1]
	fmla	v11.2d, v29.2d, v26.d[1]
	fmla	v12.2d, v28.2d, v27.d[0]
	fmla	v13.2d, v29.2d, v27.d[0]
	add		x11, x11, x12

	ldp		q24, q25, [x9, #(0*8+1*32)] // A
	ldr		d26, [x11, #(1*8)] // B
	ldr		d27, [x11, #(2*8)] // B
	fmla	v2.2d, v24.2d, v26.d[0]
	fmla	v3.2d, v25.2d, v26.d[0]
	fmla	v4.2d, v24.2d, v27.d[0]
	fmla	v5.2d, v25.2d, v27.d[0]
	ldp		q28, q29, [x14, #(0*8+1*32)] // A
	fmla	v10.2d, v28.2d, v26.d[0]
	fmla	v11.2d, v29.2d, v26.d[0]
	fmla	v12.2d, v28.2d, v27.d[0]
	fmla	v13.2d, v29.2d, v27.d[0]
	add		x11, x11, x12

	ldp		q24, q25, [x9, #(0*8+2*32)] // A
	ldr		d27, [x11, #(2*8)] // B
	fmla	v4.2d, v24.2d, v27.d[0]
	fmla	v5.2d, v25.2d, v27.d[0]
	ldp		q28, q29, [x14, #(0*8+2*32)] // A
	fmla	v12.2d, v28.2d, v27.d[0]
	fmla	v13.2d, v29.2d, v27.d[0]
	add		x11, x11, x12

	sub		w8, w8, #3
	add		x9, x9, #96

	b		0f

1:

	cmp		w13, #2
	blt		1f

	ldp		q24, q25, [x9, #(0*8+0*32)] // A
	ldr		q26, [x11, #(0*8)] // B
	fmla	v0.2d, v24.2d, v26.d[0]
	fmla	v1.2d, v25.2d, v26.d[0]
	fmla	v2.2d, v24.2d, v26.d[1]
	fmla	v3.2d, v25.2d, v26.d[1]
	ldp		q28, q29, [x14, #(0*8+0*32)] // A
	fmla	v8.2d, v28.2d, v26.d[0]
	fmla	v9.2d, v29.2d, v26.d[0]
	fmla	v10.2d, v28.2d, v26.d[1]
	fmla	v11.2d, v29.2d, v26.d[1]
	add		x11, x11, x12

	ldp		q24, q25, [x9, #(0*8+1*32)] // A
	ldr		d26, [x11, #(1*8)] // B
	fmla	v2.2d, v24.2d, v26.d[0]
	fmla	v3.2d, v25.2d, v26.d[0]
	ldp		q28, q29, [x14, #(0*8+1*32)] // A
	fmla	v10.2d, v28.2d, v26.d[0]
	fmla	v11.2d, v29.2d, v26.d[0]
	add		x11, x11, x12

	sub		w8, w8, #2
	add		x9, x9, #64

	b		0f

1:

//	cmp		w13, #1
//	blt		0f

	ldp		q24, q25, [x9, #(0*8+0*32)] // A
	ldr		d26, [x11, #(0*8)] // B
	fmla	v0.2d, v24.2d, v26.d[0]
	fmla	v1.2d, v25.2d, v26.d[0]
	ldp		q28, q29, [x14, #(0*8+0*32)] // A
	fmla	v8.2d, v28.2d, v26.d[0]
	fmla	v9.2d, v29.2d, v26.d[0]
	add		x11, x11, x12

	sub		w8, w8, #2
	add		x9, x9, #32

	b		0f

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trmm_nt_rl_8x4_vs_lib4c)
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
	.macro INNER_EDGE_TRSM_LLN_ONE_8X4_LIBC
#else
	.align 4
	FUN_START(inner_edge_trsm_lln_one_8x4_libc)
#endif

	ldp		q24, q25, [x8, #0] // E0[0+4*0]
	ldp		q26, q27, [x8, #32] // E1[0+4*0]
	add		x8, x8, x9
	ins		v24.d[0], xzr
	fmls	v0.2d, v24.2d, v0.d[0]
	fmls	v1.2d, v25.2d, v0.d[0]
	fmls	v8.2d, v26.2d, v0.d[0]
	fmls	v9.2d, v27.2d, v0.d[0]
	fmls	v2.2d, v24.2d, v2.d[0]
	fmls	v3.2d, v25.2d, v2.d[0]
	fmls	v10.2d, v26.2d, v2.d[0]
	fmls	v11.2d, v27.2d, v2.d[0]
	fmls	v4.2d, v24.2d, v4.d[0]
	fmls	v5.2d, v25.2d, v4.d[0]
	fmls	v12.2d, v26.2d, v4.d[0]
	fmls	v13.2d, v27.2d, v4.d[0]
	fmls	v6.2d, v24.2d, v6.d[0]
	fmls	v7.2d, v25.2d, v6.d[0]
	fmls	v14.2d, v26.2d, v6.d[0]
	fmls	v15.2d, v27.2d, v6.d[0]

	ldr		q25, [x8, #16] // E[2+4*1]
	ldp		q26, q27, [x8, #32] // E1[0+4*1]
	add		x8, x8, x9
	fmls	v1.2d, v25.2d, v0.d[1]
	fmls	v8.2d, v26.2d, v0.d[1]
	fmls	v9.2d, v27.2d, v0.d[1]
	fmls	v3.2d, v25.2d, v2.d[1]
	fmls	v10.2d, v26.2d, v2.d[1]
	fmls	v11.2d, v27.2d, v2.d[1]
	fmls	v5.2d, v25.2d, v4.d[1]
	fmls	v12.2d, v26.2d, v4.d[1]
	fmls	v13.2d, v27.2d, v4.d[1]
	fmls	v7.2d, v25.2d, v6.d[1]
	fmls	v14.2d, v26.2d, v6.d[1]
	fmls	v15.2d, v27.2d, v6.d[1]

	ldr		q25, [x8, #16] // E[2+4*2]
	ldp		q26, q27, [x8, #32] // E1[0+4*2]
	add		x8, x8, x9
	ins		v25.d[0], xzr
	fmls	v1.2d, v25.2d, v1.d[0]
	fmls	v8.2d, v26.2d, v1.d[0]
	fmls	v9.2d, v27.2d, v1.d[0]
	fmls	v3.2d, v25.2d, v3.d[0]
	fmls	v10.2d, v26.2d, v3.d[0]
	fmls	v11.2d, v27.2d, v3.d[0]
	fmls	v5.2d, v25.2d, v5.d[0]
	fmls	v12.2d, v26.2d, v5.d[0]
	fmls	v13.2d, v27.2d, v5.d[0]
	fmls	v7.2d, v25.2d, v7.d[0]
	fmls	v14.2d, v26.2d, v7.d[0]
	fmls	v15.2d, v27.2d, v7.d[0]

	ldp		q26, q27, [x8, #32] // E1[0+4*3]
	add		x8, x8, x9
	fmls	v8.2d, v26.2d, v1.d[1]
	fmls	v9.2d, v27.2d, v1.d[1]
	fmls	v10.2d, v26.2d, v3.d[1]
	fmls	v11.2d, v27.2d, v3.d[1]
	fmls	v12.2d, v26.2d, v5.d[1]
	fmls	v13.2d, v27.2d, v5.d[1]
	fmls	v14.2d, v26.2d, v7.d[1]
	fmls	v15.2d, v27.2d, v7.d[1]


	ldp		q24, q25, [x8, #32] // E1[0+4*4]
	ins		v24.d[0], xzr
	add		x8, x8, x9
	fmls	v8.2d, v24.2d, v8.d[0]
	fmls	v9.2d, v25.2d, v8.d[0]
	fmls	v10.2d, v24.2d, v10.d[0]
	fmls	v11.2d, v25.2d, v10.d[0]
	fmls	v12.2d, v24.2d, v12.d[0]
	fmls	v13.2d, v25.2d, v12.d[0]
	fmls	v14.2d, v24.2d, v14.d[0]
	fmls	v15.2d, v25.2d, v14.d[0]

	ldr		q25, [x8, #48] // E1[2+4*5]
	add		x8, x8, x9
	fmls	v9.2d, v25.2d, v8.d[1]
	fmls	v11.2d, v25.2d, v10.d[1]
	fmls	v13.2d, v25.2d, v12.d[1]
	fmls	v15.2d, v25.2d, v14.d[1]

	ldr		q25, [x8, #48] // E1[2+4*6]
//	add		x8, x8, x9
	ins		v25.d[0], xzr
	fmls	v9.2d, v25.2d, v9.d[0]
	fmls	v11.2d, v25.2d, v11.d[0]
	fmls	v13.2d, v25.2d, v13.d[0]
	fmls	v15.2d, v25.2d, v15.d[0]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsm_lln_one_8x4_libc)
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
	.macro INNER_EDGE_TRSM_LLN_ONE_8X4_VS_LIBC
#else
	.align 4
	FUN_START(inner_edge_trsm_lln_one_8x4_vs_libc)
#endif

	// TODO no need to check for sizes equal or smaller than 4 !!!!!!!

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
	sub		x8, x8, x9, lsl #1

	cmp		w10, #5
	blt		0f

	bgt		1f

	// 5x5
	ldr		d26, [x8, #32] // E[4+4*0]
	fmls	v8.2d, v26.2d, v0.d[0]
	fmls	v10.2d, v26.2d, v2.d[0]
	fmls	v12.2d, v26.2d, v4.d[0]
	fmls	v14.2d, v26.2d, v6.d[0]
	add		x8, x8, x9
	ldr		d26, [x8, #32] // E[4+4*1]
	fmls	v8.2d, v26.2d, v0.d[1]
	fmls	v10.2d, v26.2d, v2.d[1]
	fmls	v12.2d, v26.2d, v4.d[1]
	fmls	v14.2d, v26.2d, v6.d[1]
	add		x8, x8, x9
	ldr		d26, [x8, #32] // E[4+4*2]
	fmls	v8.2d, v26.2d, v1.d[0]
	fmls	v10.2d, v26.2d, v3.d[0]
	fmls	v12.2d, v26.2d, v5.d[0]
	fmls	v14.2d, v26.2d, v7.d[0]
	add		x8, x8, x9
	ldr		d26, [x8, #32] // E[4+4*3]
	fmls	v8.2d, v26.2d, v1.d[1]
	fmls	v10.2d, v26.2d, v3.d[1]
	fmls	v12.2d, v26.2d, v5.d[1]
	fmls	v14.2d, v26.2d, v7.d[1]
//	sub		x8, x8, x9, lsl #1
//	sub		x8, x8, x9
	b		0f

1:

	// 6x6
	ldr		q26, [x8, #32] // E[4+4*0]
	fmls	v8.2d, v26.2d, v0.d[0]
	fmls	v10.2d, v26.2d, v2.d[0]
	fmls	v12.2d, v26.2d, v4.d[0]
	fmls	v14.2d, v26.2d, v6.d[0]
	add		x8, x8, x9
	ldr		q26, [x8, #32] // E[4+4*1]
	fmls	v8.2d, v26.2d, v0.d[1]
	fmls	v10.2d, v26.2d, v2.d[1]
	fmls	v12.2d, v26.2d, v4.d[1]
	fmls	v14.2d, v26.2d, v6.d[1]
	add		x8, x8, x9
	ldr		q26, [x8, #32] // E[4+4*2]
	fmls	v8.2d, v26.2d, v1.d[0]
	fmls	v10.2d, v26.2d, v3.d[0]
	fmls	v12.2d, v26.2d, v5.d[0]
	fmls	v14.2d, v26.2d, v7.d[0]
	add		x8, x8, x9
	ldr		q26, [x8, #32] // E[4+4*3]
	fmls	v8.2d, v26.2d, v1.d[1]
	fmls	v10.2d, v26.2d, v3.d[1]
	fmls	v12.2d, v26.2d, v5.d[1]
	fmls	v14.2d, v26.2d, v7.d[1]
	add		x8, x8, x9
	ldr		q26, [x8, #32] // E[4+4*4]
	ins		v26.d[0], xzr
	fmls	v8.2d, v26.2d, v8.d[0]
	fmls	v10.2d, v26.2d, v10.d[0]
	fmls	v12.2d, v26.2d, v12.d[0]
	fmls	v14.2d, v26.2d, v14.d[0]
	sub		x8, x8, x9, lsl #2

	cmp		w10, #7
	blt		0f

	bgt		1f

	// 7x7
	ldr		d27, [x8, #48] // E[6+4*0]
	fmls	v9.2d, v27.2d, v0.d[0]
	fmls	v11.2d, v27.2d, v2.d[0]
	fmls	v13.2d, v27.2d, v4.d[0]
	fmls	v15.2d, v27.2d, v6.d[0]
	add		x8, x8, x9
	ldr		d27, [x8, #48] // E[6+4*1]
	fmls	v9.2d, v27.2d, v0.d[1]
	fmls	v11.2d, v27.2d, v2.d[1]
	fmls	v13.2d, v27.2d, v4.d[1]
	fmls	v15.2d, v27.2d, v6.d[1]
	add		x8, x8, x9
	ldr		d27, [x8, #48] // E[6+4*2]
	fmls	v9.2d, v27.2d, v1.d[0]
	fmls	v11.2d, v27.2d, v3.d[0]
	fmls	v13.2d, v27.2d, v5.d[0]
	fmls	v15.2d, v27.2d, v7.d[0]
	add		x8, x8, x9
	ldr		d27, [x8, #48] // E[6+4*3]
	fmls	v9.2d, v27.2d, v1.d[1]
	fmls	v11.2d, v27.2d, v3.d[1]
	fmls	v13.2d, v27.2d, v5.d[1]
	fmls	v15.2d, v27.2d, v7.d[1]
	add		x8, x8, x9
	ldr		d27, [x8, #48] // E[6+4*4]
	fmls	v9.2d, v27.2d, v8.d[0]
	fmls	v11.2d, v27.2d, v10.d[0]
	fmls	v13.2d, v27.2d, v12.d[0]
	fmls	v15.2d, v27.2d, v14.d[0]
	add		x8, x8, x9
	ldr		d27, [x8, #48] // E[6+4*5]
	fmls	v9.2d, v27.2d, v8.d[1]
	fmls	v11.2d, v27.2d, v10.d[1]
	fmls	v13.2d, v27.2d, v12.d[1]
	fmls	v15.2d, v27.2d, v14.d[1]
//	sub		x8, x8, x9, lsl #2
//	sub		x8, x8, x9
	b		0f

1:

	// 8x8
	ldr		q27, [x8, #48] // E[6+4*0]
	fmls	v9.2d, v27.2d, v0.d[0]
	fmls	v11.2d, v27.2d, v2.d[0]
	fmls	v13.2d, v27.2d, v4.d[0]
	fmls	v15.2d, v27.2d, v6.d[0]
	add		x8, x8, x9
	ldr		q27, [x8, #48] // E[6+4*1]
	fmls	v9.2d, v27.2d, v0.d[1]
	fmls	v11.2d, v27.2d, v2.d[1]
	fmls	v13.2d, v27.2d, v4.d[1]
	fmls	v15.2d, v27.2d, v6.d[1]
	add		x8, x8, x9
	ldr		q27, [x8, #48] // E[6+4*2]
	fmls	v9.2d, v27.2d, v1.d[0]
	fmls	v11.2d, v27.2d, v3.d[0]
	fmls	v13.2d, v27.2d, v5.d[0]
	fmls	v15.2d, v27.2d, v7.d[0]
	add		x8, x8, x9
	ldr		q27, [x8, #48] // E[6+4*3]
	fmls	v9.2d, v27.2d, v1.d[1]
	fmls	v11.2d, v27.2d, v3.d[1]
	fmls	v13.2d, v27.2d, v5.d[1]
	fmls	v15.2d, v27.2d, v7.d[1]
	add		x8, x8, x9
	ldr		q27, [x8, #48] // E[6+4*4]
	fmls	v9.2d, v27.2d, v8.d[0]
	fmls	v11.2d, v27.2d, v10.d[0]
	fmls	v13.2d, v27.2d, v12.d[0]
	fmls	v15.2d, v27.2d, v14.d[0]
	add		x8, x8, x9
	ldr		q27, [x8, #48] // E[6+4*5]
	fmls	v9.2d, v27.2d, v8.d[1]
	fmls	v11.2d, v27.2d, v10.d[1]
	fmls	v13.2d, v27.2d, v12.d[1]
	fmls	v15.2d, v27.2d, v14.d[1]
	add		x8, x8, x9
	ldr		q27, [x8, #48] // E[6+4*6]
	ins		v27.d[0], xzr
	fmls	v9.2d, v27.2d, v9.d[0]
	fmls	v11.2d, v27.2d, v11.d[0]
	fmls	v13.2d, v27.2d, v13.d[0]
	fmls	v15.2d, v27.2d, v15.d[0]
//	sub		x8, x8, x9, lsl #2
//	sub		x8, x8, x9, lsl #1

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsm_lln_one_8x4_vs_libc)
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
	.macro INNER_EDGE_TRSM_RLN_INV_8X4_LIB
#else
	.align 4
	FUN_START(inner_edge_trsm_rln_inv_8x4_lib)
#endif
	
	add			x11, x8, x9
	add			x12, x11, x9

	ldr			d16, [x10, #24] // E_inv[3]
	fmul		v6.2d, v6.2d, v16.d[0]
	fmul		v7.2d, v7.2d, v16.d[0]
	fmul		v14.2d, v14.2d, v16.d[0]
	fmul		v15.2d, v15.2d, v16.d[0]
	ldr			d16, [x12, #24] // E[3+4*2]
	fmls		v4.2d, v6.2d, v16.d[0]
	fmls		v5.2d, v7.2d, v16.d[0]
	fmls		v12.2d, v14.2d, v16.d[0]
	fmls		v13.2d, v15.2d, v16.d[0]
	ldr			d16, [x11, #24] // E[3+4*1]
	fmls		v2.2d, v6.2d, v16.d[0]
	fmls		v3.2d, v7.2d, v16.d[0]
	fmls		v10.2d, v14.2d, v16.d[0]
	fmls		v11.2d, v15.2d, v16.d[0]
	ldr			d16, [x8, #24] // E[3+4*0]
	fmls		v0.2d, v6.2d, v16.d[0]
	fmls		v1.2d, v7.2d, v16.d[0]
	fmls		v8.2d, v14.2d, v16.d[0]
	fmls		v9.2d, v15.2d, v16.d[0]

	ldr			d16, [x10, #16] // E_inv[2]
	fmul		v4.2d, v4.2d, v16.d[0]
	fmul		v5.2d, v5.2d, v16.d[0]
	fmul		v12.2d, v12.2d, v16.d[0]
	fmul		v13.2d, v13.2d, v16.d[0]
	ldr			d16, [x11, #16] // E[2+4*1]
	fmls		v2.2d, v4.2d, v16.d[0]
	fmls		v3.2d, v5.2d, v16.d[0]
	fmls		v10.2d, v12.2d, v16.d[0]
	fmls		v11.2d, v13.2d, v16.d[0]
	ldr			d16, [x8, #16] // E[2+4*0]
	fmls		v0.2d, v4.2d, v16.d[0]
	fmls		v1.2d, v5.2d, v16.d[0]
	fmls		v8.2d, v12.2d, v16.d[0]
	fmls		v9.2d, v13.2d, v16.d[0]

	ldr			d16, [x10, #8] // E_inv[1]
	fmul		v2.2d, v2.2d, v16.d[0]
	fmul		v3.2d, v3.2d, v16.d[0]
	fmul		v10.2d, v10.2d, v16.d[0]
	fmul		v11.2d, v11.2d, v16.d[0]
	ldr			d16, [x8, #8] // E[1+4*0]
	fmls		v0.2d, v2.2d, v16.d[0]
	fmls		v1.2d, v3.2d, v16.d[0]
	fmls		v8.2d, v10.2d, v16.d[0]
	fmls		v9.2d, v11.2d, v16.d[0]

	ldr			d16, [x10, #0] // E_inv[0]
	fmul		v0.2d, v0.2d, v16.d[0]
	fmul		v1.2d, v1.2d, v16.d[0]
	fmul		v8.2d, v8.2d, v16.d[0]
	fmul		v9.2d, v9.2d, v16.d[0]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsm_rln_inv_8x4_lib)
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
	.macro INNER_EDGE_TRSM_RLN_INV_8X4_VS_LIB
#else
	.align 4
	FUN_START(inner_edge_trsm_rln_inv_8x4_vs_lib)
#endif
	
	add			x12, x8, x9
	add			x13, x12, x9

	cmp		w11, #3
	ble		1f

	ldr			d16, [x10, #24] // E_inv[3]
	fmul		v6.2d, v6.2d, v16.d[0]
	fmul		v7.2d, v7.2d, v16.d[0]
	fmul		v14.2d, v14.2d, v16.d[0]
	fmul		v15.2d, v15.2d, v16.d[0]
	ldr			d16, [x13, #24] // E[3+4*2]
	fmls		v4.2d, v6.2d, v16.d[0]
	fmls		v5.2d, v7.2d, v16.d[0]
	fmls		v12.2d, v14.2d, v16.d[0]
	fmls		v13.2d, v15.2d, v16.d[0]
	ldr			d16, [x12, #24] // E[3+4*1]
	fmls		v2.2d, v6.2d, v16.d[0]
	fmls		v3.2d, v7.2d, v16.d[0]
	fmls		v10.2d, v14.2d, v16.d[0]
	fmls		v11.2d, v15.2d, v16.d[0]
	ldr			d16, [x8, #24] // E[3+4*0]
	fmls		v0.2d, v6.2d, v16.d[0]
	fmls		v1.2d, v7.2d, v16.d[0]
	fmls		v8.2d, v14.2d, v16.d[0]
	fmls		v9.2d, v15.2d, v16.d[0]

1:
	cmp		w11, #2
	ble		1f

	ldr			d16, [x10, #16] // E_inv[2]
	fmul		v4.2d, v4.2d, v16.d[0]
	fmul		v5.2d, v5.2d, v16.d[0]
	fmul		v12.2d, v12.2d, v16.d[0]
	fmul		v13.2d, v13.2d, v16.d[0]
	ldr			d16, [x12, #16] // E[2+4*1]
	fmls		v2.2d, v4.2d, v16.d[0]
	fmls		v3.2d, v5.2d, v16.d[0]
	fmls		v10.2d, v12.2d, v16.d[0]
	fmls		v11.2d, v13.2d, v16.d[0]
	ldr			d16, [x8, #16] // E[2+4*0]
	fmls		v0.2d, v4.2d, v16.d[0]
	fmls		v1.2d, v5.2d, v16.d[0]
	fmls		v8.2d, v12.2d, v16.d[0]
	fmls		v9.2d, v13.2d, v16.d[0]

1:
	cmp		w11, #1
	ble		1f

	ldr			d16, [x10, #8] // E_inv[1]
	fmul		v2.2d, v2.2d, v16.d[0]
	fmul		v3.2d, v3.2d, v16.d[0]
	fmul		v10.2d, v10.2d, v16.d[0]
	fmul		v11.2d, v11.2d, v16.d[0]
	ldr			d16, [x8, #8] // E[1+4*0]
	fmls		v0.2d, v2.2d, v16.d[0]
	fmls		v1.2d, v3.2d, v16.d[0]
	fmls		v8.2d, v10.2d, v16.d[0]
	fmls		v9.2d, v11.2d, v16.d[0]

1:

	ldr			d16, [x10, #0] // E_inv[0]
	fmul		v0.2d, v0.2d, v16.d[0]
	fmul		v1.2d, v1.2d, v16.d[0]
	fmul		v8.2d, v8.2d, v16.d[0]
	fmul		v9.2d, v9.2d, v16.d[0]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsm_rln_inv_8x4_vs_lib)
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
	
	ldr			d16, [x10, #0] // E_inv[0]
	fmul		v0.2d, v0.2d, v16.d[0]
	fmul		v1.2d, v1.2d, v16.d[0]
	fmul		v8.2d, v8.2d, v16.d[0]
	fmul		v9.2d, v9.2d, v16.d[0]
	ldr			d16, [x8, #8] // E[1+4*0]
	fmls		v2.2d, v0.2d, v16.d[0]
	fmls		v3.2d, v1.2d, v16.d[0]
	fmls		v10.2d, v8.2d, v16.d[0]
	fmls		v11.2d, v9.2d, v16.d[0]
	ldr			d16, [x8, #16] // E[2+4*0]
	fmls		v4.2d, v0.2d, v16.d[0]
	fmls		v5.2d, v1.2d, v16.d[0]
	fmls		v12.2d, v8.2d, v16.d[0]
	fmls		v13.2d, v9.2d, v16.d[0]
	ldr			d16, [x8, #24] // E[3+4*0]
	fmls		v6.2d, v0.2d, v16.d[0]
	fmls		v7.2d, v1.2d, v16.d[0]
	fmls		v14.2d, v8.2d, v16.d[0]
	fmls		v15.2d, v9.2d, v16.d[0]
	add			x8, x8, x9

	ldr			d16, [x10, #8] // E_inv[1]
	fmul		v2.2d, v2.2d, v16.d[0]
	fmul		v3.2d, v3.2d, v16.d[0]
	fmul		v10.2d, v10.2d, v16.d[0]
	fmul		v11.2d, v11.2d, v16.d[0]
	ldr			d16, [x8, #16] // E[2+4*1]
	fmls		v4.2d, v2.2d, v16.d[0]
	fmls		v5.2d, v3.2d, v16.d[0]
	fmls		v12.2d, v10.2d, v16.d[0]
	fmls		v13.2d, v11.2d, v16.d[0]
	ldr			d16, [x8, #24] // E[3+4*1]
	fmls		v6.2d, v2.2d, v16.d[0]
	fmls		v7.2d, v3.2d, v16.d[0]
	fmls		v14.2d, v10.2d, v16.d[0]
	fmls		v15.2d, v11.2d, v16.d[0]
	add			x8, x8, x9

	ldr			d16, [x10, #16] // E_inv[2]
	fmul		v4.2d, v4.2d, v16.d[0]
	fmul		v5.2d, v5.2d, v16.d[0]
	fmul		v12.2d, v12.2d, v16.d[0]
	fmul		v13.2d, v13.2d, v16.d[0]
	ldr			d16, [x8, #24] // E[3+4*1]
	fmls		v6.2d, v4.2d, v16.d[0]
	fmls		v7.2d, v5.2d, v16.d[0]
	fmls		v14.2d, v12.2d, v16.d[0]
	fmls		v15.2d, v13.2d, v16.d[0]
//	add			x8, x8, x9

	ldr			d16, [x10, #24] // E_inv[2]
	fmul		v6.2d, v6.2d, v16.d[0]
	fmul		v7.2d, v7.2d, v16.d[0]
	fmul		v14.2d, v14.2d, v16.d[0]
	fmul		v15.2d, v15.2d, v16.d[0]
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
	ldr			d16, [x10, #0] // E_inv[0]
	fmul		v0.2d, v0.2d, v16.d[0]
	fmul		v1.2d, v1.2d, v16.d[0]
	fmul		v8.2d, v8.2d, v16.d[0]
	fmul		v9.2d, v9.2d, v16.d[0]
	cmp			w11, #2
	blt			0f // return

	// second column
	ldr			d16, [x8, #8] // E[1+4*0]
	fmls		v2.2d, v0.2d, v16.d[0]
	fmls		v3.2d, v1.2d, v16.d[0]
	fmls		v10.2d, v8.2d, v16.d[0]
	fmls		v11.2d, v9.2d, v16.d[0]
	ldr			d16, [x10, #8] // E_inv[1]
	fmul		v2.2d, v2.2d, v16.d[0]
	fmul		v3.2d, v3.2d, v16.d[0]
	fmul		v10.2d, v10.2d, v16.d[0]
	fmul		v11.2d, v11.2d, v16.d[0]
	cmp			w11, #3
	blt			0f // return

	// third column
	add			x12, x8, x9
	ldr			d16, [x8, #16] // E[2+4*0]
	fmls		v4.2d, v0.2d, v16.d[0]
	fmls		v5.2d, v1.2d, v16.d[0]
	fmls		v12.2d, v8.2d, v16.d[0]
	fmls		v13.2d, v9.2d, v16.d[0]
	ldr			d16, [x12, #16] // E[2+4*1]
	fmls		v4.2d, v2.2d, v16.d[0]
	fmls		v5.2d, v3.2d, v16.d[0]
	fmls		v12.2d, v10.2d, v16.d[0]
	fmls		v13.2d, v11.2d, v16.d[0]
	ldr			d16, [x10, #16] // E_inv[2]
	fmul		v4.2d, v4.2d, v16.d[0]
	fmul		v5.2d, v5.2d, v16.d[0]
	fmul		v12.2d, v12.2d, v16.d[0]
	fmul		v13.2d, v13.2d, v16.d[0]
	cmp			w11, #4
	blt			0f // return

	// forth column
	add			x13, x12, x9
	ldr			d16, [x8, #24] // E[3+4*0]
	fmls		v6.2d, v0.2d, v16.d[0]
	fmls		v7.2d, v1.2d, v16.d[0]
	fmls		v14.2d, v8.2d, v16.d[0]
	fmls		v15.2d, v9.2d, v16.d[0]
	ldr			d16, [x12, #24] // E[3+4*1]
	fmls		v6.2d, v2.2d, v16.d[0]
	fmls		v7.2d, v3.2d, v16.d[0]
	fmls		v14.2d, v10.2d, v16.d[0]
	fmls		v15.2d, v11.2d, v16.d[0]
	ldr			d16, [x13, #24] // E[3+4*2]
	fmls		v6.2d, v4.2d, v16.d[0]
	fmls		v7.2d, v5.2d, v16.d[0]
	fmls		v14.2d, v12.2d, v16.d[0]
	fmls		v15.2d, v13.2d, v16.d[0]
	ldr			d16, [x10, #24] // E_inv[3]
	fmul		v6.2d, v6.2d, v16.d[0]
	fmul		v7.2d, v7.2d, v16.d[0]
	fmul		v14.2d, v14.2d, v16.d[0]
	fmul		v15.2d, v15.2d, v16.d[0]

0:
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsm_rlt_inv_8x4_vs_lib)
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
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRSM_RLT_ONE_8X4_LIB
#else
	.align 4
	FUN_START(inner_edge_trsm_rlt_one_8x4_lib)
#endif
	
	ldr			d16, [x8, #8] // E[1+4*0]
	fmls		v2.2d, v0.2d, v16.d[0]
	fmls		v3.2d, v1.2d, v16.d[0]
	fmls		v10.2d, v8.2d, v16.d[0]
	fmls		v11.2d, v9.2d, v16.d[0]
	ldr			d16, [x8, #16] // E[2+4*0]
	fmls		v4.2d, v0.2d, v16.d[0]
	fmls		v5.2d, v1.2d, v16.d[0]
	fmls		v12.2d, v8.2d, v16.d[0]
	fmls		v13.2d, v9.2d, v16.d[0]
	ldr			d16, [x8, #24] // E[3+4*0]
	fmls		v6.2d, v0.2d, v16.d[0]
	fmls		v7.2d, v1.2d, v16.d[0]
	fmls		v14.2d, v8.2d, v16.d[0]
	fmls		v15.2d, v9.2d, v16.d[0]
	add			x8, x8, x9

	ldr			d16, [x8, #16] // E[2+4*1]
	fmls		v4.2d, v2.2d, v16.d[0]
	fmls		v5.2d, v3.2d, v16.d[0]
	fmls		v12.2d, v10.2d, v16.d[0]
	fmls		v13.2d, v11.2d, v16.d[0]
	ldr			d16, [x8, #24] // E[3+4*1]
	fmls		v6.2d, v2.2d, v16.d[0]
	fmls		v7.2d, v3.2d, v16.d[0]
	fmls		v14.2d, v10.2d, v16.d[0]
	fmls		v15.2d, v11.2d, v16.d[0]
	add			x8, x8, x9

	ldr			d16, [x8, #24] // E[3+4*1]
	fmls		v6.2d, v4.2d, v16.d[0]
	fmls		v7.2d, v5.2d, v16.d[0]
	fmls		v14.2d, v12.2d, v16.d[0]
	fmls		v15.2d, v13.2d, v16.d[0]
//	add			x8, x8, x9

//	add			x8, x8, x9

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsm_rlt_one_8x4_lib)
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
// w10  <- n1
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRSM_RLT_ONE_8X4_VS_LIB
#else
	.align 4
	FUN_START(inner_edge_trsm_rlt_one_8x4_vs_lib)
#endif
	
	// first column
	cmp			w10, #2
	blt			0f // return

	// second column
	ldr			d16, [x8, #8] // E[1+4*0]
	fmls		v2.2d, v0.2d, v16.d[0]
	fmls		v3.2d, v1.2d, v16.d[0]
	fmls		v10.2d, v8.2d, v16.d[0]
	fmls		v11.2d, v9.2d, v16.d[0]
	cmp			w10, #3
	blt			0f // return

	// third column
	add			x12, x8, x9
	ldr			d16, [x8, #16] // E[2+4*0]
	fmls		v4.2d, v0.2d, v16.d[0]
	fmls		v5.2d, v1.2d, v16.d[0]
	fmls		v12.2d, v8.2d, v16.d[0]
	fmls		v13.2d, v9.2d, v16.d[0]
	ldr			d16, [x12, #16] // E[2+4*1]
	fmls		v4.2d, v2.2d, v16.d[0]
	fmls		v5.2d, v3.2d, v16.d[0]
	fmls		v12.2d, v10.2d, v16.d[0]
	fmls		v13.2d, v11.2d, v16.d[0]
	cmp			w10, #4
	blt			0f // return

	// forth column
	add			x13, x12, x9
	ldr			d16, [x8, #24] // E[3+4*0]
	fmls		v6.2d, v0.2d, v16.d[0]
	fmls		v7.2d, v1.2d, v16.d[0]
	fmls		v14.2d, v8.2d, v16.d[0]
	fmls		v15.2d, v9.2d, v16.d[0]
	ldr			d16, [x12, #24] // E[3+4*1]
	fmls		v6.2d, v2.2d, v16.d[0]
	fmls		v7.2d, v3.2d, v16.d[0]
	fmls		v14.2d, v10.2d, v16.d[0]
	fmls		v15.2d, v11.2d, v16.d[0]
	ldr			d16, [x13, #24] // E[3+4*2]
	fmls		v6.2d, v4.2d, v16.d[0]
	fmls		v7.2d, v5.2d, v16.d[0]
	fmls		v14.2d, v12.2d, v16.d[0]
	fmls		v15.2d, v13.2d, v16.d[0]

0:
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsm_rlt_one_8x4_vs_lib)
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
	.macro INNER_EDGE_TRSM_RUN_INV_8X4_LIB
#else
	.align 4
	FUN_START(inner_edge_trsm_run_inv_8x4_lib)
#endif
	
	add			x11, x8, x9
	add			x12, x11, x9
	add			x13, x12, x9

	ldr			d16, [x10, #0] // E_inv[0]
	fmul		v0.2d, v0.2d, v16.d[0]
	fmul		v1.2d, v1.2d, v16.d[0]
	fmul		v8.2d, v8.2d, v16.d[0]
	fmul		v9.2d, v9.2d, v16.d[0]
	ldr			d16, [x11, #0] // E[0+4*1]
	fmls		v2.2d, v0.2d, v16.d[0]
	fmls		v3.2d, v1.2d, v16.d[0]
	fmls		v10.2d, v8.2d, v16.d[0]
	fmls		v11.2d, v9.2d, v16.d[0]
	ldr			d16, [x12, #0] // E[0+4*2]
	fmls		v4.2d, v0.2d, v16.d[0]
	fmls		v5.2d, v1.2d, v16.d[0]
	fmls		v12.2d, v8.2d, v16.d[0]
	fmls		v13.2d, v9.2d, v16.d[0]
	ldr			d16, [x13, #0] // E[0+4*3]
	fmls		v6.2d, v0.2d, v16.d[0]
	fmls		v7.2d, v1.2d, v16.d[0]
	fmls		v14.2d, v8.2d, v16.d[0]
	fmls		v15.2d, v9.2d, v16.d[0]

	ldr			d16, [x10, #8] // E_inv[1]
	fmul		v2.2d, v2.2d, v16.d[0]
	fmul		v3.2d, v3.2d, v16.d[0]
	fmul		v10.2d, v10.2d, v16.d[0]
	fmul		v11.2d, v11.2d, v16.d[0]
	ldr			d16, [x12, #8] // E[1+4*2]
	fmls		v4.2d, v2.2d, v16.d[0]
	fmls		v5.2d, v3.2d, v16.d[0]
	fmls		v12.2d, v10.2d, v16.d[0]
	fmls		v13.2d, v11.2d, v16.d[0]
	ldr			d16, [x13, #8] // E[1+4*3]
	fmls		v6.2d, v2.2d, v16.d[0]
	fmls		v7.2d, v3.2d, v16.d[0]
	fmls		v14.2d, v10.2d, v16.d[0]
	fmls		v15.2d, v11.2d, v16.d[0]

	ldr			d16, [x10, #16] // E_inv[2]
	fmul		v4.2d, v4.2d, v16.d[0]
	fmul		v5.2d, v5.2d, v16.d[0]
	fmul		v12.2d, v12.2d, v16.d[0]
	fmul		v13.2d, v13.2d, v16.d[0]
	ldr			d16, [x13, #16] // E[2+4*3]
	fmls		v6.2d, v4.2d, v16.d[0]
	fmls		v7.2d, v5.2d, v16.d[0]
	fmls		v14.2d, v12.2d, v16.d[0]
	fmls		v15.2d, v13.2d, v16.d[0]

	ldr			d16, [x10, #24] // E_inv[2]
	fmul		v6.2d, v6.2d, v16.d[0]
	fmul		v7.2d, v7.2d, v16.d[0]
	fmul		v14.2d, v14.2d, v16.d[0]
	fmul		v15.2d, v15.2d, v16.d[0]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsm_run_inv_8x4_lib)
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
	.macro INNER_EDGE_TRSM_RUN_INV_8X4_VS_LIB
#else
	.align 4
	FUN_START(inner_edge_trsm_run_inv_8x4_vs_lib)
#endif
	
	// first column
	ldr			d16, [x10, #0] // E_inv[0]
	fmul		v0.2d, v0.2d, v16.d[0]
	fmul		v1.2d, v1.2d, v16.d[0]
	fmul		v8.2d, v8.2d, v16.d[0]
	fmul		v9.2d, v9.2d, v16.d[0]
	cmp			w11, #2
	blt			0f // return

	// second column
	add			x8, x8, x9
	ldr			d16, [x8, #0] // E[0+4*1]
	fmls		v2.2d, v0.2d, v16.d[0]
	fmls		v3.2d, v1.2d, v16.d[0]
	fmls		v10.2d, v8.2d, v16.d[0]
	fmls		v11.2d, v9.2d, v16.d[0]
	ldr			d16, [x10, #8] // E_inv[1]
	fmul		v2.2d, v2.2d, v16.d[0]
	fmul		v3.2d, v3.2d, v16.d[0]
	fmul		v10.2d, v10.2d, v16.d[0]
	fmul		v11.2d, v11.2d, v16.d[0]
	cmp			w11, #3
	blt			0f // return

	// third column
	add			x8, x8, x9
	ldr			d16, [x8, #0] // E[0+4*2]
	fmls		v4.2d, v0.2d, v16.d[0]
	fmls		v5.2d, v1.2d, v16.d[0]
	fmls		v12.2d, v8.2d, v16.d[0]
	fmls		v13.2d, v9.2d, v16.d[0]
	ldr			d16, [x8, #8] // E[1+4*2]
	fmls		v4.2d, v2.2d, v16.d[0]
	fmls		v5.2d, v3.2d, v16.d[0]
	fmls		v12.2d, v10.2d, v16.d[0]
	fmls		v13.2d, v11.2d, v16.d[0]
	ldr			d16, [x10, #16] // E_inv[2]
	fmul		v4.2d, v4.2d, v16.d[0]
	fmul		v5.2d, v5.2d, v16.d[0]
	fmul		v12.2d, v12.2d, v16.d[0]
	fmul		v13.2d, v13.2d, v16.d[0]
	cmp			w11, #4
	blt			0f // return

	// forth column
	add			x8, x8, x9
	ldr			d16, [x8, #0] // E[0+4*3]
	fmls		v6.2d, v0.2d, v16.d[0]
	fmls		v7.2d, v1.2d, v16.d[0]
	fmls		v14.2d, v8.2d, v16.d[0]
	fmls		v15.2d, v9.2d, v16.d[0]
	ldr			d16, [x8, #8] // E[1+4*3]
	fmls		v6.2d, v2.2d, v16.d[0]
	fmls		v7.2d, v3.2d, v16.d[0]
	fmls		v14.2d, v10.2d, v16.d[0]
	fmls		v15.2d, v11.2d, v16.d[0]
	ldr			d16, [x8, #16] // E[2+4*3]
	fmls		v6.2d, v4.2d, v16.d[0]
	fmls		v7.2d, v5.2d, v16.d[0]
	fmls		v14.2d, v12.2d, v16.d[0]
	fmls		v15.2d, v13.2d, v16.d[0]
	ldr			d16, [x10, #24] // E_inv[3]
	fmul		v6.2d, v6.2d, v16.d[0]
	fmul		v7.2d, v7.2d, v16.d[0]
	fmul		v14.2d, v14.2d, v16.d[0]
	fmul		v15.2d, v15.2d, v16.d[0]

0:
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsm_run_inv_8x4_vs_lib)
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

	trn1	v24.2d, v8.2d, v10.2d
	trn2	v10.2d, v8.2d, v10.2d
	trn1	v25.2d, v13.2d, v15.2d
	trn2	v15.2d, v13.2d, v15.2d
	trn1	v26.2d, v9.2d, v11.2d
	trn2	v27.2d, v9.2d, v11.2d
	trn1	v9.2d, v12.2d, v14.2d
	trn2	v11.2d, v12.2d, v14.2d
	mov		v8.16b, v24.16b
	mov		v13.16b, v25.16b
	mov		v12.16b, v26.16b
	mov		v14.16b, v27.16b

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
// x11  <- ldc*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_8X4_LIB
#else
	.align	4
	FUN_START(inner_scale_ab_8x4_lib)
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
	fmul	v8.2d, v8.2d, v28.d[0]
	fmul	v9.2d, v9.2d, v28.d[0]
	fmul	v10.2d, v10.2d, v28.d[0]
	fmul	v11.2d, v11.2d, v28.d[0]
	fmul	v12.2d, v12.2d, v28.d[0]
	fmul	v13.2d, v13.2d, v28.d[0]
	fmul	v14.2d, v14.2d, v28.d[0]
	fmul	v15.2d, v15.2d, v28.d[0]

	fcmpe	d29, #0.0
	beq		0f

	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	add		x10, x10, x11
	fmla	v0.2d, v24.2d, v29.d[0]
	fmla	v1.2d, v25.2d, v29.d[0]
	fmla	v8.2d, v26.2d, v29.d[0]
	fmla	v9.2d, v27.2d, v29.d[0]

	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	add		x10, x10, x11
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	fmla	v11.2d, v27.2d, v29.d[0]

	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	add		x10, x10, x11
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]
	fmla	v12.2d, v26.2d, v29.d[0]
	fmla	v13.2d, v27.2d, v29.d[0]

	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	add		x10, x10, x11
	fmla	v6.2d, v24.2d, v29.d[0]
	fmla	v7.2d, v25.2d, v29.d[0]
	fmla	v14.2d, v26.2d, v29.d[0]
	fmla	v15.2d, v27.2d, v29.d[0]

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
// x11  <- ldc*sizeof(double)
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
	fmul	v8.2d, v8.2d, v28.d[0]
	fmul	v9.2d, v9.2d, v28.d[0]
	fmul	v10.2d, v10.2d, v28.d[0]
	fmul	v11.2d, v11.2d, v28.d[0]
	fmul	v12.2d, v12.2d, v28.d[0]
	fmul	v13.2d, v13.2d, v28.d[0]
	fmul	v14.2d, v14.2d, v28.d[0]
	fmul	v15.2d, v15.2d, v28.d[0]

	fcmpe	d29, #0.0
	beq		0f

	cmp		w12, #4
	blt		1f

	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	add		x10, x10, x11
	fmla	v0.2d, v24.2d, v29.d[0]
	fmla	v1.2d, v25.2d, v29.d[0]
	fmla	v8.2d, v26.2d, v29.d[0]
	fmla	v9.2d, v27.2d, v29.d[0]

	cmp		w13, #1
	ble		0f

	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	add		x10, x10, x11
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	fmla	v11.2d, v27.2d, v29.d[0]

	cmp		w13, #2
	ble		0f

	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	add		x10, x10, x11
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]
	fmla	v12.2d, v26.2d, v29.d[0]
	fmla	v13.2d, v27.2d, v29.d[0]

	cmp		w13, #3
	ble		0f

	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	add		x10, x10, x11
	fmla	v6.2d, v24.2d, v29.d[0]
	fmla	v7.2d, v25.2d, v29.d[0]
	fmla	v14.2d, v26.2d, v29.d[0]
	fmla	v15.2d, v27.2d, v29.d[0]

	b 0f

1:
	cmp		w12, #3
	blt		2f

	ldp		q24, q25, [x10, #0]
	ldr		q26, [x10, #32]
	ldr		d27, [x10, #48]
	add		x10, x10, x11
	fmla	v0.2d, v24.2d, v29.d[0]
	fmla	v1.2d, v25.2d, v29.d[0]
	fmla	v8.2d, v26.2d, v29.d[0]
	fmla	v9.2d, v27.2d, v29.d[0]

	cmp		w13, #1
	ble		0f

	ldp		q24, q25, [x10, #0]
	ldr		q26, [x10, #32]
	ldr		d27, [x10, #48]
	add		x10, x10, x11
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	fmla	v11.2d, v27.2d, v29.d[0]

	cmp		w13, #2
	ble		0f

	ldp		q24, q25, [x10, #0]
	ldr		q26, [x10, #32]
	ldr		d27, [x10, #48]
	add		x10, x10, x11
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]
	fmla	v12.2d, v26.2d, v29.d[0]
	fmla	v13.2d, v27.2d, v29.d[0]

	cmp		w13, #3
	ble		0f

	ldp		q24, q25, [x10, #0]
	ldr		q26, [x10, #32]
	ldr		d27, [x10, #48]
	add		x10, x10, x11
	fmla	v6.2d, v24.2d, v29.d[0]
	fmla	v7.2d, v25.2d, v29.d[0]
	fmla	v14.2d, v26.2d, v29.d[0]
	fmla	v15.2d, v27.2d, v29.d[0]

	b 0f

2:
	cmp		w12, #2
	blt		3f

	ldp		q24, q25, [x10, #0]
	ldr		q26, [x10, #32]
	add		x10, x10, x11
	fmla	v0.2d, v24.2d, v29.d[0]
	fmla	v1.2d, v25.2d, v29.d[0]
	fmla	v8.2d, v26.2d, v29.d[0]

	cmp		w13, #1
	ble		0f

	ldp		q24, q25, [x10, #0]
	ldr		q26, [x10, #32]
	add		x10, x10, x11
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]

	cmp		w13, #2
	ble		0f

	ldp		q24, q25, [x10, #0]
	ldr		q26, [x10, #32]
	add		x10, x10, x11
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]
	fmla	v12.2d, v26.2d, v29.d[0]

	cmp		w13, #3
	ble		0f

	ldp		q24, q25, [x10, #0]
	ldr		q26, [x10, #32]
	add		x10, x10, x11
	fmla	v6.2d, v24.2d, v29.d[0]
	fmla	v7.2d, v25.2d, v29.d[0]
	fmla	v14.2d, v26.2d, v29.d[0]

	b 0f

3:
	cmp		w12, #1
	blt		0f

	ldp		q24, q25, [x10, #0]
	ldr		d26, [x10, #32]
	add		x10, x10, x11
	fmla	v0.2d, v24.2d, v29.d[0]
	fmla	v1.2d, v25.2d, v29.d[0]
	fmla	v8.2d, v26.2d, v29.d[0]

	cmp		w13, #1
	ble		0f

	ldp		q24, q25, [x10, #0]
	ldr		d26, [x10, #32]
	add		x10, x10, x11
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]

	cmp		w13, #2
	ble		0f

	ldp		q24, q25, [x10, #0]
	ldr		d26, [x10, #32]
	add		x10, x10, x11
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]
	fmla	v12.2d, v26.2d, v29.d[0]

	cmp		w13, #3
	ble		0f

	ldp		q24, q25, [x10, #0]
	ldr		q26, [x10, #32]
	add		x10, x10, x11
	fmla	v6.2d, v24.2d, v29.d[0]
	fmla	v7.2d, v25.2d, v29.d[0]
	fmla	v14.2d, v26.2d, v29.d[0]

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
// x8  <- beta
// x9  <- C
// x10 <- ldc
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M1B_8X4_LIB
#else
	.align	4
	FUN_START(inner_scale_m1b_8x4_lib)
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

	fneg	v8.2d, v8.2d
	fneg	v9.2d, v9.2d
	fneg	v10.2d, v10.2d
	fneg	v11.2d, v11.2d

	fneg	v12.2d, v12.2d
	fneg	v13.2d, v13.2d
	fneg	v14.2d, v14.2d
	fneg	v15.2d, v15.2d

	fcmpe	d29, #0.0
	beq		0f

	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v24.2d, v29.d[0]
	fmla	v1.2d, v25.2d, v29.d[0]
	fmla	v8.2d, v26.2d, v29.d[0]
	fmla	v9.2d, v27.2d, v29.d[0]

	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	add		x9, x9, x10
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	fmla	v11.2d, v27.2d, v29.d[0]

	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	add		x9, x9, x10
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]
	fmla	v12.2d, v26.2d, v29.d[0]
	fmla	v13.2d, v27.2d, v29.d[0]

	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	add		x9, x9, x10
	fmla	v6.2d, v24.2d, v29.d[0]
	fmla	v7.2d, v25.2d, v29.d[0]
	fmla	v14.2d, v26.2d, v29.d[0]
	fmla	v15.2d, v27.2d, v29.d[0]

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
// x19  <- ldc*sizeof(double)
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

	ld1		{v29.2d}, [x8]

	fneg	v0.2d, v0.2d
	fneg	v1.2d, v1.2d
	fneg	v2.2d, v2.2d
	fneg	v3.2d, v3.2d

	fneg	v4.2d, v4.2d
	fneg	v5.2d, v5.2d
	fneg	v6.2d, v6.2d
	fneg	v7.2d, v7.2d

	fneg	v8.2d, v8.2d
	fneg	v9.2d, v9.2d
	fneg	v10.2d, v10.2d
	fneg	v11.2d, v11.2d

	fneg	v12.2d, v12.2d
	fneg	v13.2d, v13.2d
	fneg	v14.2d, v14.2d
	fneg	v15.2d, v15.2d

	fcmpe	d29, #0.0
	beq		0f

	cmp		w11, #4
	blt		1f

	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v24.2d, v29.d[0]
	fmla	v1.2d, v25.2d, v29.d[0]
	fmla	v8.2d, v26.2d, v29.d[0]
	fmla	v9.2d, v27.2d, v29.d[0]

	cmp		w12, #1
	ble		0f

	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	add		x9, x9, x10
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	fmla	v11.2d, v27.2d, v29.d[0]

	cmp		w12, #2
	ble		0f

	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	add		x9, x9, x10
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]
	fmla	v12.2d, v26.2d, v29.d[0]
	fmla	v13.2d, v27.2d, v29.d[0]

	cmp		w12, #3
	ble		0f

	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	add		x9, x9, x10
	fmla	v6.2d, v24.2d, v29.d[0]
	fmla	v7.2d, v25.2d, v29.d[0]
	fmla	v14.2d, v26.2d, v29.d[0]
	fmla	v15.2d, v27.2d, v29.d[0]

	b 0f

1:
	cmp		w11, #3
	blt		2f

	ldp		q24, q25, [x9, #0]
	ldr		q26, [x9, #32]
	ldr		d27, [x9, #48]
	add		x9, x9, x10
	fmla	v0.2d, v24.2d, v29.d[0]
	fmla	v1.2d, v25.2d, v29.d[0]
	fmla	v8.2d, v26.2d, v29.d[0]
	fmla	v9.2d, v27.2d, v29.d[0]

	cmp		w12, #1
	ble		0f

	ldp		q24, q25, [x9, #0]
	ldr		q26, [x9, #32]
	ldr		d27, [x9, #48]
	add		x9, x9, x10
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	fmla	v11.2d, v27.2d, v29.d[0]

	cmp		w12, #2
	ble		0f

	ldp		q24, q25, [x9, #0]
	ldr		q26, [x9, #32]
	ldr		d27, [x9, #48]
	add		x9, x9, x10
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]
	fmla	v12.2d, v26.2d, v29.d[0]
	fmla	v13.2d, v27.2d, v29.d[0]

	cmp		w12, #3
	ble		0f

	ldp		q24, q25, [x9, #0]
	ldr		q26, [x9, #32]
	ldr		d27, [x9, #48]
	add		x9, x9, x10
	fmla	v6.2d, v24.2d, v29.d[0]
	fmla	v7.2d, v25.2d, v29.d[0]
	fmla	v14.2d, v26.2d, v29.d[0]
	fmla	v15.2d, v27.2d, v29.d[0]

	b 0f

2:
	cmp		w11, #2
	blt		3f

	ldp		q24, q25, [x9, #0]
	ldr		q26, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v24.2d, v29.d[0]
	fmla	v1.2d, v25.2d, v29.d[0]
	fmla	v8.2d, v26.2d, v29.d[0]

	cmp		w12, #1
	ble		0f

	ldp		q24, q25, [x9, #0]
	ldr		q26, [x9, #32]
	add		x9, x9, x10
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]

	cmp		w12, #2
	ble		0f

	ldp		q24, q25, [x9, #0]
	ldr		q26, [x9, #32]
	add		x9, x9, x10
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]
	fmla	v12.2d, v26.2d, v29.d[0]

	cmp		w12, #3
	ble		0f

	ldp		q24, q25, [x9, #0]
	ldr		q26, [x9, #32]
	add		x9, x9, x10
	fmla	v6.2d, v24.2d, v29.d[0]
	fmla	v7.2d, v25.2d, v29.d[0]
	fmla	v14.2d, v26.2d, v29.d[0]

	b 0f

3:
	cmp		w11, #1
	blt		0f

	ldp		q24, q25, [x9, #0]
	ldr		d26, [x9, #32]
	add		x9, x9, x10
	fmla	v0.2d, v24.2d, v29.d[0]
	fmla	v1.2d, v25.2d, v29.d[0]
	fmla	v8.2d, v26.2d, v29.d[0]

	cmp		w12, #1
	ble		0f

	ldp		q24, q25, [x9, #0]
	ldr		d26, [x9, #32]
	add		x9, x9, x10
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]

	cmp		w12, #2
	ble		0f

	ldp		q24, q25, [x9, #0]
	ldr		d26, [x9, #32]
	add		x9, x9, x10
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]
	fmla	v12.2d, v26.2d, v29.d[0]

	cmp		w12, #3
	ble		0f

	ldp		q24, q25, [x9, #0]
	ldr		d26, [x9, #32]
	add		x9, x9, x10
	fmla	v6.2d, v24.2d, v29.d[0]
	fmla	v7.2d, v25.2d, v29.d[0]
	fmla	v14.2d, v26.2d, v29.d[0]

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
// x9  <- ldc*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M11_8X4_LIB
#else
	.align	4
	FUN_START(inner_scale_m11_8x4_lib)
#endif

	ldp		q24, q25, [x8, #0]
	ldp		q26, q27, [x8, #32]
	add		x8, x8, x9
	fsub	v0.2d, v24.2d, v0.2d
	fsub	v1.2d, v25.2d, v1.2d
	fsub	v8.2d, v26.2d, v8.2d
	fsub	v9.2d, v27.2d, v9.2d

	ldp		q24, q25, [x8, #0]
	ldp		q26, q27, [x8, #32]
	add		x8, x8, x9
	fsub	v2.2d, v24.2d, v2.2d
	fsub	v3.2d, v25.2d, v3.2d
	fsub	v10.2d, v26.2d, v10.2d
	fsub	v11.2d, v27.2d, v11.2d

	ldp		q24, q25, [x8, #0]
	ldp		q26, q27, [x8, #32]
	add		x8, x8, x9
	fsub	v4.2d, v24.2d, v4.2d
	fsub	v5.2d, v25.2d, v5.2d
	fsub	v12.2d, v26.2d, v12.2d
	fsub	v13.2d, v27.2d, v13.2d

	ldp		q24, q25, [x8, #0]
	ldp		q26, q27, [x8, #32]
	add		x8, x8, x9
	fsub	v6.2d, v24.2d, v6.2d
	fsub	v7.2d, v25.2d, v7.2d
	fsub	v14.2d, v26.2d, v14.2d
	fsub	v15.2d, v27.2d, v15.2d

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m11_8x4_lib)
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
	.macro INNER_SCALE_M11_8X4_VS_LIB
#else
	.align	4
	FUN_START(inner_scale_m11_8x4_vs_lib)
#endif

	cmp		w10, #4
	blt		1f

	ldp		q24, q25, [x8, #0]
	ldp		q26, q27, [x8, #32]
	add		x8, x8, x9
	fsub	v0.2d, v24.2d, v0.2d
	fsub	v1.2d, v25.2d, v1.2d
	fsub	v8.2d, v26.2d, v8.2d
	fsub	v9.2d, v27.2d, v9.2d

	cmp		w11, #1
	ble		0f

	ldp		q24, q25, [x8, #0]
	ldp		q26, q27, [x8, #32]
	add		x8, x8, x9
	fsub	v2.2d, v24.2d, v2.2d
	fsub	v3.2d, v25.2d, v3.2d
	fsub	v10.2d, v26.2d, v10.2d
	fsub	v11.2d, v27.2d, v11.2d

	cmp		w11, #2
	ble		0f

	ldp		q24, q25, [x8, #0]
	ldp		q26, q27, [x8, #32]
	add		x8, x8, x9
	fsub	v4.2d, v24.2d, v4.2d
	fsub	v5.2d, v25.2d, v5.2d
	fsub	v12.2d, v26.2d, v12.2d
	fsub	v13.2d, v27.2d, v13.2d

	cmp		w11, #3
	ble		0f

	ldp		q24, q25, [x8, #0]
	ldp		q26, q27, [x8, #32]
	add		x8, x8, x9
	fsub	v6.2d, v24.2d, v6.2d
	fsub	v7.2d, v25.2d, v7.2d
	fsub	v14.2d, v26.2d, v14.2d
	fsub	v15.2d, v27.2d, v15.2d

	b 0f

1:
	cmp		w10, #3
	blt		2f

	ldp		q24, q25, [x8, #0]
	ldr		q26, [x8, #32]
	ldr		d27, [x8, #48]
	add		x8, x8, x9
	fsub	v0.2d, v24.2d, v0.2d
	fsub	v1.2d, v25.2d, v1.2d
	fsub	v8.2d, v26.2d, v8.2d
	fsub	v9.2d, v27.2d, v9.2d

	cmp		w11, #1
	ble		0f

	ldp		q24, q25, [x8, #0]
	ldr		q26, [x8, #32]
	ldr		d27, [x8, #48]
	add		x8, x8, x9
	fsub	v2.2d, v24.2d, v2.2d
	fsub	v3.2d, v25.2d, v3.2d
	fsub	v10.2d, v26.2d, v10.2d
	fsub	v11.2d, v27.2d, v11.2d

	cmp		w11, #2
	ble		0f

	ldp		q24, q25, [x8, #0]
	ldr		q26, [x8, #32]
	ldr		d27, [x8, #48]
	add		x8, x8, x9
	fsub	v4.2d, v24.2d, v4.2d
	fsub	v5.2d, v25.2d, v5.2d
	fsub	v12.2d, v26.2d, v12.2d
	fsub	v13.2d, v27.2d, v13.2d

	cmp		w11, #3
	ble		0f

	ldp		q24, q25, [x8, #0]
	ldr		q26, [x8, #32]
	ldr		d27, [x8, #48]
	add		x8, x8, x9
	fsub	v6.2d, v24.2d, v6.2d
	fsub	v7.2d, v25.2d, v7.2d
	fsub	v14.2d, v26.2d, v14.2d
	fsub	v15.2d, v27.2d, v15.2d

	b 0f

2:
	cmp		w10, #2
	blt		3f

	ldp		q24, q25, [x8, #0]
	ldr		q26, [x8, #32]
	add		x8, x8, x9
	fsub	v0.2d, v24.2d, v0.2d
	fsub	v1.2d, v25.2d, v1.2d
	fsub	v8.2d, v26.2d, v8.2d

	cmp		w11, #1
	ble		0f

	ldp		q24, q25, [x8, #0]
	ldr		q26, [x8, #32]
	add		x8, x8, x9
	fsub	v2.2d, v24.2d, v2.2d
	fsub	v3.2d, v25.2d, v3.2d
	fsub	v10.2d, v26.2d, v10.2d

	cmp		w11, #2
	ble		0f

	ldp		q24, q25, [x8, #0]
	ldr		q26, [x8, #32]
	add		x8, x8, x9
	fsub	v4.2d, v24.2d, v4.2d
	fsub	v5.2d, v25.2d, v5.2d
	fsub	v12.2d, v26.2d, v12.2d

	cmp		w11, #3
	ble		0f

	ldp		q24, q25, [x8, #0]
	ldr		q26, [x8, #32]
	add		x8, x8, x9
	fsub	v6.2d, v24.2d, v6.2d
	fsub	v7.2d, v25.2d, v7.2d
	fsub	v14.2d, v26.2d, v14.2d

	b 0f

3:
	cmp		w10, #1
	blt		0f

	ldp		q24, q25, [x8, #0]
	ldr		d26, [x8, #32]
	add		x8, x8, x9
	fsub	v0.2d, v24.2d, v0.2d
	fsub	v1.2d, v25.2d, v1.2d
	fsub	v8.2d, v26.2d, v8.2d

	cmp		w11, #1
	ble		0f

	ldp		q24, q25, [x8, #0]
	ldr		d26, [x8, #32]
	add		x8, x8, x9
	fsub	v2.2d, v24.2d, v2.2d
	fsub	v3.2d, v25.2d, v3.2d
	fsub	v10.2d, v26.2d, v10.2d

	cmp		w11, #2
	ble		0f

	ldp		q24, q25, [x8, #0]
	ldr		d26, [x8, #32]
	add		x8, x8, x9
	fsub	v4.2d, v24.2d, v4.2d
	fsub	v5.2d, v25.2d, v5.2d
	fsub	v12.2d, v26.2d, v12.2d

	cmp		w11, #3
	ble		0f

	ldp		q24, q25, [x8, #0]
	ldr		d26, [x8, #32]
	add		x8, x8, x9
	fsub	v6.2d, v24.2d, v6.2d
	fsub	v7.2d, v25.2d, v7.2d
	fsub	v14.2d, v26.2d, v14.2d

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m11_8x4_vs_lib)
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
	.macro INNER_SCALE_AB_4X8_LIB
#else
	.align	4
	FUN_START(inner_scale_ab_4x8_lib)
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
	fmul	v8.2d, v8.2d, v28.d[0]
	fmul	v9.2d, v9.2d, v28.d[0]
	fmul	v10.2d, v10.2d, v28.d[0]
	fmul	v11.2d, v11.2d, v28.d[0]
	fmul	v12.2d, v12.2d, v28.d[0]
	fmul	v13.2d, v13.2d, v28.d[0]
	fmul	v14.2d, v14.2d, v28.d[0]
	fmul	v15.2d, v15.2d, v28.d[0]

	fcmpe	d29, #0.0
	beq		0f

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	fmla	v0.2d, v24.2d, v29.d[0]
	fmla	v1.2d, v25.2d, v29.d[0]

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	fmla	v6.2d, v24.2d, v29.d[0]
	fmla	v7.2d, v25.2d, v29.d[0]

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	fmla	v8.2d, v24.2d, v29.d[0]
	fmla	v9.2d, v25.2d, v29.d[0]

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	fmla	v10.2d, v24.2d, v29.d[0]
	fmla	v11.2d, v25.2d, v29.d[0]

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	fmla	v12.2d, v24.2d, v29.d[0]
	fmla	v13.2d, v25.2d, v29.d[0]

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	fmla	v14.2d, v24.2d, v29.d[0]
	fmla	v15.2d, v25.2d, v29.d[0]

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
// x11  <- ldc*sizeof(double)
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
	fmul	v8.2d, v8.2d, v28.d[0]
	fmul	v9.2d, v9.2d, v28.d[0]
	fmul	v10.2d, v10.2d, v28.d[0]
	fmul	v11.2d, v11.2d, v28.d[0]
	fmul	v12.2d, v12.2d, v28.d[0]
	fmul	v13.2d, v13.2d, v28.d[0]
	fmul	v14.2d, v14.2d, v28.d[0]
	fmul	v15.2d, v15.2d, v28.d[0]

	fcmpe	d29, #0.0
	beq		0f

	cmp		w12, #4
	blt		1f

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	fmla	v0.2d, v24.2d, v29.d[0]
	fmla	v1.2d, v25.2d, v29.d[0]

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	fmla	v6.2d, v24.2d, v29.d[0]
	fmla	v7.2d, v25.2d, v29.d[0]

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	fmla	v8.2d, v24.2d, v29.d[0]
	fmla	v9.2d, v25.2d, v29.d[0]

	cmp		w13, #5
	ble		0f

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	fmla	v10.2d, v24.2d, v29.d[0]
	fmla	v11.2d, v25.2d, v29.d[0]

	cmp		w13, #6
	ble		0f

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	fmla	v12.2d, v24.2d, v29.d[0]
	fmla	v13.2d, v25.2d, v29.d[0]

	cmp		w13, #7
	ble		0f

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	fmla	v14.2d, v24.2d, v29.d[0]
	fmla	v15.2d, v25.2d, v29.d[0]

	b 0f

1:
	cmp		w12, #3
	blt		2f

	ldr		q24, [x10, #0]
	ldr		d25, [x10, #16]
	add		x10, x10, x11
	fmla	v0.2d, v24.2d, v29.d[0]
	fmla	v1.2d, v25.2d, v29.d[0]

	ldr		q24, [x10, #0]
	ldr		d25, [x10, #16]
	add		x10, x10, x11
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]

	ldr		q24, [x10, #0]
	ldr		d25, [x10, #16]
	add		x10, x10, x11
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]

	ldr		q24, [x10, #0]
	ldr		d25, [x10, #16]
	add		x10, x10, x11
	fmla	v6.2d, v24.2d, v29.d[0]
	fmla	v7.2d, v25.2d, v29.d[0]

	ldr		q24, [x10, #0]
	ldr		d25, [x10, #16]
	add		x10, x10, x11
	fmla	v8.2d, v24.2d, v29.d[0]
	fmla	v9.2d, v25.2d, v29.d[0]

	cmp		w13, #5
	ble		0f

	ldr		q24, [x10, #0]
	ldr		d25, [x10, #16]
	add		x10, x10, x11
	fmla	v10.2d, v24.2d, v29.d[0]
	fmla	v11.2d, v25.2d, v29.d[0]

	cmp		w13, #6
	ble		0f

	ldr		q24, [x10, #0]
	ldr		d25, [x10, #16]
	add		x10, x10, x11
	fmla	v12.2d, v24.2d, v29.d[0]
	fmla	v13.2d, v25.2d, v29.d[0]

	cmp		w13, #7
	ble		0f

	ldr		q24, [x10, #0]
	ldr		d25, [x10, #16]
	add		x10, x10, x11
	fmla	v14.2d, v24.2d, v29.d[0]
	fmla	v15.2d, v25.2d, v29.d[0]

	b 0f

2:
	cmp		w12, #2
	blt		3f

	ldr		q24, [x10, #0]
	add		x10, x10, x11
	fmla	v0.2d, v24.2d, v29.d[0]

	ldr		q24, [x10, #0]
	add		x10, x10, x11
	fmla	v2.2d, v24.2d, v29.d[0]

	ldr		q24, [x10, #0]
	add		x10, x10, x11
	fmla	v4.2d, v24.2d, v29.d[0]

	ldr		q24, [x10, #0]
	add		x10, x10, x11
	fmla	v6.2d, v24.2d, v29.d[0]

	ldr		q24, [x10, #0]
	add		x10, x10, x11
	fmla	v8.2d, v24.2d, v29.d[0]

	cmp		w13, #5
	ble		0f

	ldr		q24, [x10, #0]
	add		x10, x10, x11
	fmla	v10.2d, v24.2d, v29.d[0]

	cmp		w13, #6
	ble		0f

	ldr		q24, [x10, #0]
	add		x10, x10, x11
	fmla	v12.2d, v24.2d, v29.d[0]

	cmp		w13, #7
	ble		0f

	ldr		q24, [x10, #0]
	add		x10, x10, x11
	fmla	v14.2d, v24.2d, v29.d[0]

	b 0f

3:
	cmp		w12, #1
	blt		0f

	ldr		d24, [x10, #0]
	add		x10, x10, x11
	fmla	v0.2d, v24.2d, v29.d[0]

	ldr		d24, [x10, #0]
	add		x10, x10, x11
	fmla	v2.2d, v24.2d, v29.d[0]

	ldr		d24, [x10, #0]
	add		x10, x10, x11
	fmla	v4.2d, v24.2d, v29.d[0]

	ldr		d24, [x10, #0]
	add		x10, x10, x11
	fmla	v6.2d, v24.2d, v29.d[0]

	ldr		d24, [x10, #0]
	add		x10, x10, x11
	fmla	v8.2d, v24.2d, v29.d[0]

	cmp		w13, #5
	ble		0f

	ldr		d24, [x10, #0]
	add		x10, x10, x11
	fmla	v10.2d, v24.2d, v29.d[0]

	cmp		w13, #6
	ble		0f

	ldr		d24, [x10, #0]
	add		x10, x10, x11
	fmla	v12.2d, v24.2d, v29.d[0]

	cmp		w13, #7
	ble		0f

	ldr		d24, [x10, #0]
	add		x10, x10, x11
	fmla	v14.2d, v24.2d, v29.d[0]

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
// x9   <- ldd*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_8X4_LIB
#else
	.align 4
	FUN_START(inner_store_8x4_lib)
#endif

	stp		q0, q1, [x8, #0]
	stp		q8, q9, [x8, #32]
	add		x8, x8, x9
	stp		q2, q3, [x8, #0]
	stp		q10, q11, [x8, #32]
	add		x8, x8, x9
	stp		q4, q5, [x8, #0]
	stp		q12, q13, [x8, #32]
	add		x8, x8, x9
	stp		q6, q7, [x8, #0]
	stp		q14, q15, [x8, #32]

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
// x9   <- ldd*sizeof(double)
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
	blt		1f

	// 1st col
	stp		q0, q1, [x8, #0]
	stp		q8, q9, [x8, #32]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	stp		q2, q3, [x8, #0]
	stp		q10, q11, [x8, #32]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	stp		q4, q5, [x8, #0]
	stp		q12, q13, [x8, #32]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	stp		q6, q7, [x8, #0]
	stp		q14, q15, [x8, #32]
	b		0f

1:
	cmp		w10, #7
	blt		1f

	// 1st col
	stp		q0, q1, [x8, #0]
	str		q8, [x8, #32]
	str		d9, [x8, #48]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	stp		q2, q3, [x8, #0]
	str		q10, [x8, #32]
	str		d11, [x8, #48]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	stp		q4, q5, [x8, #0]
	str		q12, [x8, #32]
	str		d13, [x8, #48]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	stp		q6, q7, [x8, #0]
	str		q14, [x8, #32]
	str		d15, [x8, #48]
	b		0f

1:
	cmp		w10, #6
	blt		1f

	// 1st col
	stp		q0, q1, [x8, #0]
	str		q8, [x8, #32]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	stp		q2, q3, [x8, #0]
	str		q10, [x8, #32]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	stp		q4, q5, [x8, #0]
	str		q12, [x8, #32]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	stp		q6, q7, [x8, #0]
	str		q14, [x8, #32]
	b		0f

1:
//	cmp		w10, #5
//	blt		0f

	// 1st col
	stp		q0, q1, [x8, #0]
	str		d8, [x8, #32]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	stp		q2, q3, [x8, #0]
	str		d10, [x8, #32]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	stp		q4, q5, [x8, #0]
	str		d12, [x8, #32]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	stp		q6, q7, [x8, #0]
	str		d14, [x8, #32]
//	b		0f

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
// x9   <- ldd*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_8X4_LIB
#else
	.align 4
	FUN_START(inner_store_l_8x4_lib)
#endif

	ins		v16.d[0], v2.d[1]
	ins		v17.d[0], v7.d[1]

	stp		q0, q1, [x8, #0]
	stp		q8, q9, [x8, #32]
	add		x8, x8, x9
	str		d16, [x8, #8]
	str		q3, [x8, #16]
	stp		q10, q11, [x8, #32]
	add		x8, x8, x9
	str		q5, [x8, #16]
	stp		q12, q13, [x8, #32]
	add		x8, x8, x9
	str		d17, [x8, #24]
	stp		q14, q15, [x8, #32]

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
// x9   <- ldd*sizeof(double)
// x10  <- km
// x11  <- kn
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_8X4_VS_LIB
#else
	.align 4
	FUN_START(inner_store_l_8x4_vs_lib)
#endif

	cmp		w10, #8
	blt		1f

	// 1st col
	stp		q0, q1, [x8, #0]
	stp		q8, q9, [x8, #32]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	ins		v16.d[0], v2.d[1]
	str		d16, [x8, #8]
	str		q3, [x8, #16]
	stp		q10, q11, [x8, #32]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	str		q5, [x8, #16]
	stp		q12, q13, [x8, #32]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	ins		v17.d[0], v7.d[1]
	str		d17, [x8, #24]
	stp		q14, q15, [x8, #32]
	b		0f

1:
	cmp		w10, #7
	blt		1f

	// 1st col
	stp		q0, q1, [x8, #0]
	str		q8, [x8, #32]
	str		d9, [x8, #48]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	ins		v16.d[0], v2.d[1]
	str		d16, [x8, #8]
//	str		q3, [x8, #16]
//	str		q10, [x8, #32]
	stp		q3, q10, [x8, #16]
	str		d11, [x8, #48]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
//	str		q5, [x8, #16]
//	str		q12, [x8, #32]
	stp		q5, q12, [x8, #16]
	str		d13, [x8, #48]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	ins		v17.d[0], v7.d[1]
	str		d17, [x8, #24]
	str		q14, [x8, #32]
	str		d15, [x8, #48]
	b		0f

1:
	cmp		w10, #6
	blt		1f

	// 1st col
	stp		q0, q1, [x8, #0]
	str		q8, [x8, #32]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	ins		v16.d[0], v2.d[1]
	str		d16, [x8, #8]
//	str		q3, [x8, #16]
//	str		q10, [x8, #32]
	stp		q3, q10, [x8, #16]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
//	str		q5, [x8, #16]
//	str		q12, [x8, #32]
	stp		q5, q12, [x8, #16]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	ins		v17.d[0], v7.d[1]
	str		d17, [x8, #24]
	str		q14, [x8, #32]
	b		0f

1:
//	cmp		w10, #5
//	blt		0f

	// 1st col
	stp		q0, q1, [x8, #0]
	str		d8, [x8, #32]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	ins		v16.d[0], v2.d[1]
	str		d16, [x8, #8]
	str		q3, [x8, #16]
	str		d10, [x8, #32]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	str		q5, [x8, #16]
	str		d12, [x8, #32]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	ins		v17.d[0], v7.d[1]
//	str		d17, [x8, #24]
//	str		d14, [x8, #32]
	stp		d17, d14, [x8, #24]
//	b		0f

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_8x4_vs_lib)
#endif





// subroutine
//
// input arguments:
// x8   <- D
// x9   <- ldd*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_U_8X4_LIB
#else
	.align 4
	FUN_START(inner_store_u_8x4_lib)
#endif

	stp		q0, q1, [x8, #0]
	str		d8, [x8, #32]
	add		x8, x8, x9
	stp		q2, q3, [x8, #0]
	str		q10, [x8, #32]
	add		x8, x8, x9
	stp		q4, q5, [x8, #0]
	str		q12, [x8, #32]
	str		d13, [x8, #48]
	add		x8, x8, x9
	stp		q6, q7, [x8, #0]
	stp		q14, q15, [x8, #32]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_u_8x4_lib)
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
	.macro INNER_STORE_U_8X4_VS_LIB
#else
	.align 4
	FUN_START(inner_store_u_8x4_vs_lib)
#endif

	cmp		w10, #8
	blt		1f

	// 1st col
	stp		q0, q1, [x8, #0]
	str		d8, [x8, #32]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	stp		q2, q3, [x8, #0]
	str		q10, [x8, #32]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	stp		q4, q5, [x8, #0]
	str		q12, [x8, #32]
	str		d13, [x8, #48]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	stp		q6, q7, [x8, #0]
	stp		q14, q15, [x8, #32]
	b		0f

1:
	cmp		w10, #7
	blt		1f

	// 1st col
	stp		q0, q1, [x8, #0]
	str		d8, [x8, #32]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	stp		q2, q3, [x8, #0]
	str		q10, [x8, #32]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	stp		q4, q5, [x8, #0]
	str		q12, [x8, #32]
	str		d13, [x8, #48]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	stp		q6, q7, [x8, #0]
	str		q14, [x8, #32]
	str		d15, [x8, #48]
	b		0f

1:
	cmp		w10, #6
	blt		1f

	// 1st col
	stp		q0, q1, [x8, #0]
	str		d8, [x8, #32]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	stp		q2, q3, [x8, #0]
	str		q10, [x8, #32]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	stp		q4, q5, [x8, #0]
	str		q12, [x8, #32]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	stp		q6, q7, [x8, #0]
	str		q14, [x8, #32]
	b		0f

1:
//	cmp		w10, #5
//	blt		0f

	// 1st col
	stp		q0, q1, [x8, #0]
	str		d8, [x8, #32]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	stp		q2, q3, [x8, #0]
	str		d10, [x8, #32]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	stp		q4, q5, [x8, #0]
	str		d12, [x8, #32]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	stp		q6, q7, [x8, #0]
	str		d14, [x8, #32]
//	b		0f

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_u_8x4_vs_lib)
#endif





// subroutine
//
// input arguments:
// x8   <- D
// x9   <- ldd*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X8_LIB
#else
	.align 4
	FUN_START(inner_store_4x8_lib)
#endif

	stp		q0, q1, [x8, #0]
	add		x8, x8, x9
	stp		q2, q3, [x8, #0]
	add		x8, x8, x9
	stp		q4, q5, [x8, #0]
	add		x8, x8, x9
	stp		q6, q7, [x8, #0]
	add		x8, x8, x9
	stp		q8, q9, [x8, #0]
	add		x8, x8, x9
	stp		q10, q11, [x8, #0]
	add		x8, x8, x9
	stp		q12, q13, [x8, #0]
	add		x8, x8, x9
	stp		q14, q15, [x8, #0]
	add		x8, x8, x9

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
// x9   <- ldd*sizeof(double)
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
	blt		1f

	// 1st-5th col
	stp		q0, q1, [x8, #0]
	add		x8, x8, x9
	stp		q2, q3, [x8, #0]
	add		x8, x8, x9
	stp		q4, q5, [x8, #0]
	add		x8, x8, x9
	stp		q6, q7, [x8, #0]
	add		x8, x8, x9
	stp		q8, q9, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #6
	blt		0f
	// 6th col
	stp		q10, q11, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #7
	blt		0f
	// 7th col
	stp		q12, q13, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #7
	beq		0f
	// 8th col
	stp		q14, q15, [x8, #0]
	b		0f

1:
	cmp		w10, #3
	blt		1f

	// 1st-5th col
	str		q0, [x8, #0]
	str		d1, [x8, #16]
	add		x8, x8, x9
	str		q2, [x8, #0]
	str		d3, [x8, #16]
	add		x8, x8, x9
	str		q4, [x8, #0]
	str		d5, [x8, #16]
	add		x8, x8, x9
	str		q6, [x8, #0]
	str		d7, [x8, #16]
	add		x8, x8, x9
	str		q8, [x8, #0]
	str		d9, [x8, #16]
	add		x8, x8, x9
	cmp		w11, #6
	blt		0f
	// 6th col
	str		q10, [x8, #0]
	str		d11, [x8, #16]
	add		x8, x8, x9
	cmp		w11, #7
	blt		0f
	// 7th col
	str		q12, [x8, #0]
	str		d13, [x8, #16]
	add		x8, x8, x9
	cmp		w11, #7
	beq		0f
	// 8th col
	str		q14, [x8, #0]
	str		d15, [x8, #16]
	b		0f

1:
	cmp		w10, #2
	blt		1f

	// 1st-5th col
	str		q0, [x8, #0]
	add		x8, x8, x9
	str		q2, [x8, #0]
	add		x8, x8, x9
	str		q4, [x8, #0]
	add		x8, x8, x9
	str		q6, [x8, #0]
	add		x8, x8, x9
	str		q8, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #6
	blt		0f
	// 6th col
	str		q10, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #7
	blt		0f
	// 7th col
	str		q12, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #7
	beq		0f
	// 8th col
	str		q14, [x8, #0]
	b		0f

1:
	cmp		w10, #1
	blt		0f

	// 1st-5th col
	str		d0, [x8, #0]
	add		x8, x8, x9
	str		d2, [x8, #0]
	add		x8, x8, x9
	str		d4, [x8, #0]
	add		x8, x8, x9
	str		d6, [x8, #0]
	add		x8, x8, x9
	str		d8, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #6
	blt		0f
	// 6th col
	str		d10, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #7
	blt		0f
	// 7th col
	str		d12, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #7
	beq		0f
	// 8th col
	str		d14, [x8, #0]
//	b		0f

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
// x8   <- alpha
// x9  <- D
// x10  <- ldd*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_A1_STORE_8X4_LIB
#else
	.align	4
	FUN_START(inner_scale_a1_store_8x4_lib)
#endif

	ld1		{v28.2d}, [x8]

	ldp		q24, q25, [x9, #0]
	fmla	v24.2d, v0.2d, v28.d[0]
	fmla	v25.2d, v1.2d, v28.d[0]
	ldp		q26, q27, [x9, #32]
	fmla	v26.2d, v8.2d, v28.d[0]
	fmla	v27.2d, v9.2d, v28.d[0]
	stp		q24, q25, [x9, #0]
	stp		q26, q27, [x9, #32]
	add		x9, x9, x10

	ldp		q24, q25, [x9, #0]
	fmla	v24.2d, v2.2d, v28.d[0]
	fmla	v25.2d, v3.2d, v28.d[0]
	ldp		q26, q27, [x9, #32]
	fmla	v26.2d, v10.2d, v28.d[0]
	fmla	v27.2d, v11.2d, v28.d[0]
	stp		q24, q25, [x9, #0]
	stp		q26, q27, [x9, #32]
	add		x9, x9, x10

	ldp		q24, q25, [x9, #0]
	fmla	v24.2d, v4.2d, v28.d[0]
	fmla	v25.2d, v5.2d, v28.d[0]
	ldp		q26, q27, [x9, #32]
	fmla	v26.2d, v12.2d, v28.d[0]
	fmla	v27.2d, v13.2d, v28.d[0]
	stp		q24, q25, [x9, #0]
	stp		q26, q27, [x9, #32]
	add		x9, x9, x10

	ldp		q24, q25, [x9, #0]
	fmla	v24.2d, v6.2d, v28.d[0]
	fmla	v25.2d, v7.2d, v28.d[0]
	ldp		q26, q27, [x9, #32]
	fmla	v26.2d, v14.2d, v28.d[0]
	fmla	v27.2d, v15.2d, v28.d[0]
	stp		q24, q25, [x9, #0]
	stp		q26, q27, [x9, #32]
	add		x9, x9, x10

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_a1_store_8x4_lib)
#endif





// subroutine
//
// input arguments:
// x8   <- D
// x9   <- ldd*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_PREFETCH_8X4_LIB
#else
	.align 4
	FUN_START(inner_prefetch_8x4_lib)
#endif

//	prfm	PSTL1STRM, [x8, #0]
	prfm	PLDL1KEEP, [x8, #0]
	prfm	PLDL1KEEP, [x8, #32]
	add		x8, x8, x9
	prfm	PLDL1KEEP, [x8, #0]
	prfm	PLDL1KEEP, [x8, #32]
	add		x8, x8, x9
	prfm	PLDL1KEEP, [x8, #0]
	prfm	PLDL1KEEP, [x8, #32]
	add		x8, x8, x9
	prfm	PLDL1KEEP, [x8, #0]
	prfm	PLDL1KEEP, [x8, #32]

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
// x9   <- ldd*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_PREFETCH_4X8_LIB
#else
	.align 4
	FUN_START(inner_prefetch_4x8_lib)
#endif

	prfm	PLDL1KEEP, [x8, #0]
	add		x8, x8, x9
	prfm	PLDL1KEEP, [x8, #0]
	add		x8, x8, x9
	prfm	PLDL1KEEP, [x8, #0]
	add		x8, x8, x9
	prfm	PLDL1KEEP, [x8, #0]
	add		x8, x8, x9
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

	FUN_END(inner_prefetch_4x8_lib)
#endif





//                                 w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8
// void kernel_dgemm_nt_8x4_lib44cc(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dgemm_nt_8x4_lib44cc)
	FUN_START(kernel_dgemm_nt_8x4_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// prefetch C
//	mov		x8, x5 // beta
	ld1		{v29.2d}, [x5]
	fcmpe	d29, #0.0
	beq		100f

	mov		x8, x6 // C
	mov		w9, w7 // ldc
	lsl		w9, w9, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_PREFETCH_8X4_LIB
#else
	CALL(inner_prefetch_8x4_lib)
#endif

100:


	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB
#else
	CALL(inner_scale_ab_8x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nt_8x4_lib44cc)





//                                     w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8     sp+16        sp+24
// void kernel_dgemm_nt_8x4_p0_lib44cc(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *A_p, double *B_p)

	.align	4
	GLOB(kernel_dgemm_nt_8x4_p0_lib44cc)
	FUN_START(kernel_dgemm_nt_8x4_p0_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// prefetch C
//	mov		x8, x5 // beta
	ld1		{v29.2d}, [x5]
	fcmpe	d29, #0.0
	beq		100f

	mov		x8, x6 // C
	mov		w9, w7 // ldc
	lsl		w9, w9, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_PREFETCH_8X4_LIB
#else
	CALL(inner_prefetch_8x4_lib)
#endif

100:


	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B

	ldr		x12, [sp, #(STACKSIZE + 16)] // A_p
	ldr		x13, [sp, #(STACKSIZE + 24)] // B_p

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_P0_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_p0_lib4)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB
#else
	CALL(inner_scale_ab_8x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nt_8x4_p0_lib44cc)





//                                    w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8     sp+16
// void kernel_dgemm_nt_8x4_p_lib44cc(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *A_p)

	.align	4
	GLOB(kernel_dgemm_nt_8x4_p_lib44cc)
	FUN_START(kernel_dgemm_nt_8x4_p_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// prefetch C
//	mov		x8, x5 // beta
	ld1		{v29.2d}, [x5]
	fcmpe	d29, #0.0
	beq		100f

	mov		x8, x6 // C
	mov		w9, w7 // ldc
	lsl		w9, w9, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_PREFETCH_8X4_LIB
#else
	CALL(inner_prefetch_8x4_lib)
#endif

100:


	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B

	ldr		x12, [sp, #(STACKSIZE + 16)] // A_p
	mov		x13, x4 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_PL_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_pl_lib4)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB
#else
	CALL(inner_scale_ab_8x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nt_8x4_p_lib44cc)





//                                    w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8     sp+16   sp+24
// void kernel_dgemm_nt_8x4_vs_lib44cc(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dgemm_nt_8x4_vs_lib44cc)
	FUN_START(kernel_dgemm_nt_8x4_vs_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // ldc
	lsl		w11, w11, #3 // 8*sdc
	ldr		w12, [sp, #(STACKSIZE + 16)] // m1
	ldr		w13, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_VS_LIB
#else
	CALL(inner_scale_ab_8x4_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd
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

	FUN_END(kernel_dgemm_nt_8x4_vs_lib44cc)






//                                 w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16
// void kernel_dgemm_nt_8x4_lib4ccc(int kmax, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dgemm_nt_8x4_lib4ccc)
	FUN_START(kernel_dgemm_nt_8x4_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B
	mov		w12, w5 // sda
	lsl		w12, w12, #3 // 8*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4c)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB
#else
	CALL(inner_scale_ab_8x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nt_8x4_lib4ccc)





//                                    w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16    sp+24   sp+32
// void kernel_dgemm_nt_8x4_vs_lib4ccc(int kmax, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dgemm_nt_8x4_vs_lib4ccc)
	FUN_START(kernel_dgemm_nt_8x4_vs_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B
	mov		w12, w5 // sda
	lsl		w12, w12, #3 // 8*ldb

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
	// TODO prefetch vs
//	ldr		x8, [sp, #(STACKSIZE + 8)] // D
//	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
//	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*sdc
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
	lsl		w9, w9, #3 // 8*sdd
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

	FUN_END(kernel_dgemm_nt_8x4_vs_lib4ccc)





//                                 w0        x1             x2         w3         x4       x5            x6         w7       sp+0       sp+8
// void kernel_dgemm_nt_8x4_libc4cc(int kmax, double *alpha, double *A, int lda, double *B, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dgemm_nt_8x4_libc4cc)
	FUN_START(kernel_dgemm_nt_8x4_libc4cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x4 // B
	mov		x10, x2 // A
	mov		w11, w3 // lda
	lsl		w11, w11, #3 // 8*lda

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X8_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_4x8_lib4c)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_8X4_LIB
#else
	CALL(inner_tran_8x4_lib)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB
#else
	CALL(inner_scale_ab_8x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nt_8x4_libc4cc)






//                                     w0        x1             x2         w3         x4       x5            x6         w7       sp+0       sp+8     sp+16   sp+24
// void kernel_dgemm_nt_8x4_vs_libc4cc(int kmax, double *alpha, double *A, int lda, double *B, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dgemm_nt_8x4_vs_libc4cc)
	FUN_START(kernel_dgemm_nt_8x4_vs_libc4cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x4 // B
	mov		x10, x2 // A
	mov		w11, w3 // lda
	lsl		w11, w11, #3 // 8*lda

	ldr		w13, [sp, #(STACKSIZE + 16)] // m1
	cmp		w13, #5
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X5_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_4x5_lib4c)
#endif

	b		103f

100:

	ldr		w13, [sp, #(STACKSIZE + 16)] // m1
	cmp		w13, #6
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X6_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_4x6_lib4c)
#endif
	
	b		103f

101:

	ldr		w13, [sp, #(STACKSIZE + 16)] // m1
	cmp		w13, #7
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X7_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_4x7_lib4c)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X8_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_4x8_lib4c)
#endif

103:

#if MACRO_LEVEL>=1
	INNER_TRAN_8X4_LIB
#else
	CALL(inner_tran_8x4_lib)
#endif



	// prefetch
//	ldr		x8, [sp, #(STACKSIZE + 0)] // D
//	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
//	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*sdc
	ldr		w12, [sp, #(STACKSIZE + 16)] // m1
	ldr		w13, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_VS_LIB
#else
	CALL(inner_scale_ab_8x4_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd
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

	FUN_END(kernel_dgemm_nt_8x4_vs_libc4cc)






//                                 w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16
// void kernel_dgemm_nt_8x4_libcccc(int kmax, double *alpha, double *A, int lda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dgemm_nt_8x4_libcccc)
	FUN_START(kernel_dgemm_nt_8x4_libcccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #3 // 8*sda
	mov		x11, x4 // B
	mov		w12, w5 // ldb
	lsl		w12, w12, #3 // 8*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIBCC
#else
	CALL(inner_kernel_gemm_add_nt_8x4_libcc)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB
#else
	CALL(inner_scale_ab_8x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nt_8x4_libcccc)





//                                    w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16    sp+24   sp+32
// void kernel_dgemm_nt_8x4_vs_libcccc(int kmax, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dgemm_nt_8x4_vs_libcccc)
	FUN_START(kernel_dgemm_nt_8x4_vs_libcccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #3 // 8*sda
	mov		x11, x4 // B
	mov		w12, w5 // ldb
	lsl		w12, w12, #3 // 8*ldb
	ldr		w13, [sp, #(STACKSIZE + 24)] // m1

	ldr		w14, [sp, #(STACKSIZE + 32)] // n1

	cmp		w14, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X1_VS_LIBCC
#else
	CALL(inner_kernel_gemm_add_nt_8x1_vs_libcc)
#endif

	b		103f

100:

	cmp		w14, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X2_VS_LIBCC
#else
	CALL(inner_kernel_gemm_add_nt_8x2_vs_libcc)
#endif
	
	b		103f

101:

	cmp		w14, #3
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X3_VS_LIBCC
#else
	CALL(inner_kernel_gemm_add_nt_8x3_vs_libcc)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_VS_LIBCC
#else
	CALL(inner_kernel_gemm_add_nt_8x4_vs_libcc)
#endif

103:



	// prefetch
	// TODO prefetch vs
//	ldr		x8, [sp, #(STACKSIZE + 8)] // D
//	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
//	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*sdc
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
	lsl		w9, w9, #3 // 8*sdd
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

	FUN_END(kernel_dgemm_nt_8x4_vs_libcccc)





//                                 w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16
// void kernel_dgemm_nn_8x4_lib4ccc(int kmax, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dgemm_nn_8x4_lib4ccc)
	FUN_START(kernel_dgemm_nn_8x4_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B
	mov		w12, w5 // ldb
	lsl		w12, w12, #3 // 8*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_8x4_lib4c)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB
#else
	CALL(inner_scale_ab_8x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nn_8x4_lib4ccc)





//                                    w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16    sp+24   sp+32
// void kernel_dgemm_nn_8x4_vs_lib4ccc(int kmax, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dgemm_nn_8x4_vs_lib4ccc)
	FUN_START(kernel_dgemm_nn_8x4_vs_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B
	mov		w12, w5 // ldb
	lsl		w12, w12, #3 // 8*ldb

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
	// TODO prefetch vs
//	ldr		x8, [sp, #(STACKSIZE + 8)] // D
//	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
//	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*sdc
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
	lsl		w9, w9, #3 // 8*sdd
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

	FUN_END(kernel_dgemm_nn_8x4_vs_lib4ccc)





//                                 w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16
// void kernel_dgemm_nn_8x4_libcccc(int kmax, double *alpha, double *A, int lda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dgemm_nn_8x4_libcccc)
	FUN_START(kernel_dgemm_nn_8x4_libcccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #3 // 8*sda
	mov		x11, x4 // B
	mov		w12, w5 // ldb
	lsl		w12, w12, #3 // 8*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X4_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_8x4_libcc)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB
#else
	CALL(inner_scale_ab_8x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nn_8x4_libcccc)





//                                    w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16    sp+24   sp+32
// void kernel_dgemm_nn_8x4_vs_libcccc(int kmax, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dgemm_nn_8x4_vs_libcccc)
	FUN_START(kernel_dgemm_nn_8x4_vs_libcccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #3 // 8*sda
	mov		x11, x4 // B
	mov		w12, w5 // ldb
	lsl		w12, w12, #3 // 8*ldb
	ldr		w13, [sp, #(STACKSIZE + 24)] // m1

	ldr		w14, [sp, #(STACKSIZE + 32)] // n1

	cmp		w14, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X1_VS_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_8x1_vs_libcc)
#endif

	b		103f

100:

	cmp		w14, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X2_VS_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_8x2_vs_libcc)
#endif
	
	b		103f

101:

	cmp		w14, #3
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X3_VS_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_8x3_vs_libcc)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X4_VS_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_8x4_vs_libcc)
#endif

103:



	// prefetch
	// TODO prefetch vs
//	ldr		x8, [sp, #(STACKSIZE + 8)] // D
//	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
//	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*sdc
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
	lsl		w9, w9, #3 // 8*sdd
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

	FUN_END(kernel_dgemm_nn_8x4_vs_libcccc)





//                                 w0        x1             x2         w3         x4       x5            x6         w7       sp+0       sp+8
// void kernel_dgemm_tt_8x4_libc4cc(int kmax, double *alpha, double *A, int lda, double *B, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dgemm_tt_8x4_libc4cc)
	FUN_START(kernel_dgemm_tt_8x4_libc4cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x4 // B
	mov		x10, x2 // A
	mov		w11, w3 // lda
	lsl		w11, w11, #3 // 8*lda

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X8_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_4x8_lib4c)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_8X4_LIB
#else
	CALL(inner_tran_8x4_lib)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB
#else
	CALL(inner_scale_ab_8x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_tt_8x4_libc4cc)






//                                     w0        x1             x2         w3         x4       x5            x6         w7       sp+0       sp+8     sp+16   sp+24
// void kernel_dgemm_tt_8x4_vs_libc4cc(int kmax, double *alpha, double *A, int lda, double *B, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dgemm_tt_8x4_vs_libc4cc)
	FUN_START(kernel_dgemm_tt_8x4_vs_libc4cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x4 // B
	mov		x10, x2 // A
	mov		w11, w3 // lda
	lsl		w11, w11, #3 // 8*lda

	ldr		w13, [sp, #(STACKSIZE + 16)] // m1
	cmp		w13, #5
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X5_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_4x5_lib4c)
#endif

	b		103f

100:

	ldr		w13, [sp, #(STACKSIZE + 16)] // m1
	cmp		w13, #6
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X6_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_4x6_lib4c)
#endif
	
	b		103f

101:

	ldr		w13, [sp, #(STACKSIZE + 16)] // m1
	cmp		w13, #7
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X7_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_4x7_lib4c)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X8_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_4x8_lib4c)
#endif

103:

#if MACRO_LEVEL>=1
	INNER_TRAN_8X4_LIB
#else
	CALL(inner_tran_8x4_lib)
#endif



	// prefetch
//	ldr		x8, [sp, #(STACKSIZE + 0)] // D
//	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
//	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
//	INNER_PREFETCH_4X8_LIB
#else
//	CALL(inner_prefetch_4x8_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // ldc
	lsl		w11, w11, #3 // 8*sdc
	ldr		w12, [sp, #(STACKSIZE + 16)] // m1
	ldr		w13, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_VS_LIB
#else
	CALL(inner_scale_ab_8x4_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd
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

	FUN_END(kernel_dgemm_tt_8x4_vs_libc4cc)






//                                 w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16
// void kernel_dgemm_tt_8x4_libcccc(int kmax, double *alpha, double *A, int lda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dgemm_tt_8x4_libcccc)
	FUN_START(kernel_dgemm_tt_8x4_libcccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x4 // B
	mov		w10, w5 // sdb
	lsl		w10, w10, #3 // 8*sdb
	mov		x11, x2 // A
	mov		w12, w3 // lda
	lsl		w12, w12, #3 // 8*lda

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X4_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_8x4_libcc)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_TRAN_8X4_LIB
#else
	CALL(inner_tran_8x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB
#else
	CALL(inner_scale_ab_8x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_tt_8x4_libcccc)






//                                     w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16    sp+24   sp+32
// void kernel_dgemm_tt_8x4_vs_libcccc(int kmax, double *alpha, double *A, int lda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dgemm_tt_8x4_vs_libcccc)
	FUN_START(kernel_dgemm_tt_8x4_vs_libcccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x4 // B
	mov		w10, w5 // sdb
	lsl		w10, w10, #3 // 8*sdb
	mov		x11, x2 // A
	mov		w12, w3 // lda
	lsl		w12, w12, #3 // 8*lda
	ldr		w13, [sp, #(STACKSIZE + 32)] // n1

	ldr		w14, [sp, #(STACKSIZE + 24)] // m1

	cmp		w14, #5
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X5_VS_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_4x5_vs_libcc)
#endif

	b		103f

100:

	cmp		w14, #6
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X6_VS_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_4x6_vs_libcc)
#endif
	
	b		103f

101:

	cmp		w14, #7
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X7_VS_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_4x7_vs_libcc)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X8_VS_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_4x8_vs_libcc)
#endif

103:



	// prefetch
//	ldr		x8, [sp, #(STACKSIZE + 8)] // D
//	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
//	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*sdc
	ldr		w12, [sp, #(STACKSIZE + 24)] // m1
	ldr		w13, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_TRAN_8X4_LIB
#else
	CALL(inner_tran_8x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_VS_LIB
#else
	CALL(inner_scale_ab_8x4_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*sdd
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

	FUN_END(kernel_dgemm_tt_8x4_vs_libcccc)






//                                 w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16
// void kernel_dsyrk_nt_l_8x4_lib4ccc(int kmax, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dsyrk_nt_l_8x4_lib4ccc)
	FUN_START(kernel_dsyrk_nt_l_8x4_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B
	mov		w12, w5 // sda
	lsl		w12, w12, #3 // 8*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4c)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB
#else
	CALL(inner_scale_ab_8x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X4_LIB
#else
	CALL(inner_store_l_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dsyrk_nt_l_8x4_lib4ccc)





//                                    w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16    sp+24   sp+32
// void kernel_dsyrk_nt_l_8x4_vs_lib4ccc(int kmax, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dsyrk_nt_l_8x4_vs_lib4ccc)
	FUN_START(kernel_dsyrk_nt_l_8x4_vs_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B
	mov		w12, w5 // sda
	lsl		w12, w12, #3 // 8*ldb

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
	// TODO prefetch vs
//	ldr		x8, [sp, #(STACKSIZE + 8)] // D
//	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
//	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*sdc
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
	lsl		w9, w9, #3 // 8*sdd
	ldr		w10, [sp, #(STACKSIZE + 24)] // m1
	ldr		w11, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X4_VS_LIB
#else
	CALL(inner_store_l_8x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dsyrk_nt_l_8x4_vs_lib4ccc)





//                                   w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8
// void kernel_dsyrk_nt_l_8x4_lib44cc(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dsyrk_nt_l_8x4_lib44cc)
	FUN_START(kernel_dsyrk_nt_l_8x4_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // ldc
	lsl		w11, w11, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB
#else
	CALL(inner_scale_ab_8x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X4_LIB
#else
	CALL(inner_store_l_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dsyrk_nt_l_8x4_lib44cc)





//                                      w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8     sp+16   sp+24
// void kernel_dsyrk_nt_l_8x4_vs_lib44cc(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dsyrk_nt_l_8x4_vs_lib44cc)
	FUN_START(kernel_dsyrk_nt_l_8x4_vs_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // ldc
	lsl		w11, w11, #3 // 8*sdc
	ldr		w12, [sp, #(STACKSIZE + 16)] // m1
	ldr		w13, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_VS_LIB
#else
	CALL(inner_scale_ab_8x4_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd
	ldr		w10, [sp, #(STACKSIZE + 16)] // m1
	ldr		w11, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X4_VS_LIB
#else
	CALL(inner_store_l_8x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dsyrk_nt_l_8x4_vs_lib44cc)





//                                   w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8
// void kernel_dsyrk_nt_u_8x4_lib44cc(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dsyrk_nt_u_8x4_lib44cc)
	FUN_START(kernel_dsyrk_nt_u_8x4_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // ldc
	lsl		w11, w11, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB
#else
	CALL(inner_scale_ab_8x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_U_8X4_LIB
#else
	CALL(inner_store_u_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dsyrk_nt_u_8x4_lib44cc)





//                                      w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8     sp+16   sp+24
// void kernel_dsyrk_nt_u_8x4_vs_lib44cc(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dsyrk_nt_u_8x4_vs_lib44cc)
	FUN_START(kernel_dsyrk_nt_u_8x4_vs_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // ldc
	lsl		w11, w11, #3 // 8*sdc
	ldr		w12, [sp, #(STACKSIZE + 16)] // m1
	ldr		w13, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_VS_LIB
#else
	CALL(inner_scale_ab_8x4_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd
	ldr		w10, [sp, #(STACKSIZE + 16)] // m1
	ldr		w11, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_U_8X4_VS_LIB
#else
	CALL(inner_store_u_8x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dsyrk_nt_u_8x4_vs_lib44cc)





//                                   w0     x1             x2          w3        x4         w5         x6          w7        sp+0        sp+8      sp+16         sp+24      sp+32    sp+40      sp+48
// void kernel_dger2k_nt_8x4_lib4ccc(int k, double *alpha, double *A0, int sda0, double *B0, int ldb0, double *A1, int sda1, double *B1, int ldb1, double *beta, double *C, int ldc, double *D, int ldd);

	.align	4
	GLOB(kernel_dger2k_nt_8x4_lib4ccc)
	FUN_START(kernel_dger2k_nt_8x4_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A0
	mov		w10, w3 // sda0
	lsl		w10, w10, #5 // 32*sda0
	mov		x11, x4 // B0
	mov		w12, w5 // ldb0
	lsl		w12, w12, #3 // 8*ldb0

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4c)
#endif

	mov		w8, w0 // kmax
	mov		x9, x6 // A1
	mov		w10, w7 // sda1
	lsl		w10, w10, #5 // 32*sda1
	ldr		x11, [sp, #(STACKSIZE + 0)] // B1
	ldr		w12, [sp, #(STACKSIZE + 8)] // ldb1
	lsl		w12, w12, #3 // 8*ldb1

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4c)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 40)] // D
	ldr		w9, [sp, #(STACKSIZE + 48)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_8X4_LIB
#else
	CALL(inner_prefetch_8x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	ldr		x9, [sp, #(STACKSIZE + 16)] // beta
	ldr		x10, [sp, #(STACKSIZE + 24)] // C
	ldr		w11, [sp, #(STACKSIZE + 32)] // ldc
	lsl		w11, w11, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB
#else
	CALL(inner_scale_ab_8x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 40)] // D
	ldr		w9, [sp, #(STACKSIZE + 48)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dger2k_nt_8x4_lib4ccc)





//                                   w0     x1             x2          w3        x4         w5         x6          w7        sp+0        sp+8      sp+16         sp+24      sp+32    sp+40      sp+48    sp+56   sp+64
// void kernel_dger2k_nt_8x4_lib4ccc(int k, double *alpha, double *A0, int sda0, double *B0, int ldb0, double *A1, int sda1, double *B1, int ldb1, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.align	4
	GLOB(kernel_dger2k_nt_8x4_vs_lib4ccc)
	FUN_START(kernel_dger2k_nt_8x4_vs_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	ldr		w13, [sp, #(STACKSIZE + 32)] // n1
	cmp		w13, #1
	bgt		100f

	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B
	mov		w12, w5 // sda
	lsl		w12, w12, #3 // 8*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X1_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_8x1_lib4c)
#endif

	mov		w8, w0 // kmax
	mov		x9, x6 // A1
	mov		w10, w7 // sda1
	lsl		w10, w10, #5 // 32*sda1
	ldr		x11, [sp, #(STACKSIZE + 0)] // B1
	ldr		w12, [sp, #(STACKSIZE + 8)] // ldb1
	lsl		w12, w12, #3 // 8*ldb1

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

	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B
	mov		w12, w5 // sda
	lsl		w12, w12, #3 // 8*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X2_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_8x2_lib4c)
#endif
	
	mov		w8, w0 // kmax
	mov		x9, x6 // A1
	mov		w10, w7 // sda1
	lsl		w10, w10, #5 // 32*sda1
	ldr		x11, [sp, #(STACKSIZE + 0)] // B1
	ldr		w12, [sp, #(STACKSIZE + 8)] // ldb1
	lsl		w12, w12, #3 // 8*ldb1

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

	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B
	mov		w12, w5 // sda
	lsl		w12, w12, #3 // 8*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X3_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_8x3_lib4c)
#endif
	
	mov		w8, w0 // kmax
	mov		x9, x6 // A1
	mov		w10, w7 // sda1
	lsl		w10, w10, #5 // 32*sda1
	ldr		x11, [sp, #(STACKSIZE + 0)] // B1
	ldr		w12, [sp, #(STACKSIZE + 8)] // ldb1
	lsl		w12, w12, #3 // 8*ldb1

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X3_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_8x3_lib4c)
#endif
	
	b		103f

102:

	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B
	mov		w12, w5 // sda
	lsl		w12, w12, #3 // 8*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4c)
#endif

	mov		w8, w0 // kmax
	mov		x9, x6 // A1
	mov		w10, w7 // sda1
	lsl		w10, w10, #5 // 32*sda1
	ldr		x11, [sp, #(STACKSIZE + 0)] // B1
	ldr		w12, [sp, #(STACKSIZE + 8)] // ldb1
	lsl		w12, w12, #3 // 8*ldb1

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4c)
#endif

103:



	// prefetch
	// TODO prefetch vs
//	ldr		x8, [sp, #(STACKSIZE + 40)] // D
//	ldr		w9, [sp, #(STACKSIZE + 48)] // ldd
//	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
//	INNER_PREFETCH_8X4_LIB
#else
//	CALL(inner_prefetch_8x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	ldr		x9, [sp, #(STACKSIZE + 16)] // beta
	ldr		x10, [sp, #(STACKSIZE + 24)] // C
	ldr		w11, [sp, #(STACKSIZE + 32)] // ldc
	lsl		w11, w11, #3 // 8*sdc
	ldr		w12, [sp, #(STACKSIZE + 56)] // m1
	ldr		w13, [sp, #(STACKSIZE + 64)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_VS_LIB
#else
	CALL(inner_scale_ab_8x4_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 40)] // D
	ldr		w9, [sp, #(STACKSIZE + 48)] // ldd
	lsl		w9, w9, #3 // 8*sdd
	ldr		w10, [sp, #(STACKSIZE + 56)] // m1
	ldr		w11, [sp, #(STACKSIZE + 64)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB
#else
	CALL(inner_store_8x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dger2k_nt_8x4_vs_lib4ccc)





//                                   w0     x1             x2          w3        x4          x5          w6        x7          sp+0          sp+8       sp+16    sp+24      sp+32
// void kernel_dger2k_nt_8x4_lib44cc(int k, double *alpha, double *A0, int sda0, double *B0, double *A1, int sda1, double *B1, double *beta, double *C, int ldc, double *D, int ldd);

	.align	4
	GLOB(kernel_dger2k_nt_8x4_lib44cc)
	FUN_START(kernel_dger2k_nt_8x4_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// prefetch C
	ldr		x8, [sp, #(STACKSIZE + 0)] // beta
	ld1		{v29.2d}, [x8]
	fcmpe	d29, #0.0
	beq		100f

	ldr		x8, [sp, #(STACKSIZE + 8)] // C
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldc
	lsl		w9, w9, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_PREFETCH_8X4_LIB
#else
	CALL(inner_prefetch_8x4_lib)
#endif

100:


	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A0
	mov		w10, w3 // sda0
	lsl		w10, w10, #5 // 32*sda0
	mov		x11, x4 // B0

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif

	mov		w8, w0 // kmax
	mov		x9, x5 // A1
	mov		w10, w6 // sda1
	lsl		w10, w10, #5 // 32*sda1
	mov		x11, x7 // B1

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 24)] // D
	ldr		w9, [sp, #(STACKSIZE + 32)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_8X4_LIB
#else
	CALL(inner_prefetch_8x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	ldr		x9, [sp, #(STACKSIZE + 0)] // beta
	ldr		x10, [sp, #(STACKSIZE + 8)] // C
	ldr		w11, [sp, #(STACKSIZE + 16)] // ldc
	lsl		w11, w11, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB
#else
	CALL(inner_scale_ab_8x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 24)] // D
	ldr		w9, [sp, #(STACKSIZE + 32)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dger2k_nt_8x4_lib44cc)





//                                   w0     x1             x2          w3        x4          x5          w6        x7          sp+0          sp+8       sp+16    sp+24      sp+32    sp+40   sp+48
// void kernel_dger2k_nt_8x4_lib44cc(int k, double *alpha, double *A0, int sda0, double *B0, double *A1, int sda1, double *B1, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.align	4
	GLOB(kernel_dger2k_nt_8x4_vs_lib44cc)
	FUN_START(kernel_dger2k_nt_8x4_vs_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A0
	mov		w10, w3 // sda0
	lsl		w10, w10, #5 // 32*sda0
	mov		x11, x4 // B0

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif

	mov		w8, w0 // kmax
	mov		x9, x5 // A1
	mov		w10, w6 // sda1
	lsl		w10, w10, #5 // 32*sda1
	mov		x11, x7 // B1

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	ldr		x9, [sp, #(STACKSIZE + 0)] // beta
	ldr		x10, [sp, #(STACKSIZE + 8)] // C
	ldr		w11, [sp, #(STACKSIZE + 16)] // ldc
	lsl		w11, w11, #3 // 8*sdc
	ldr		w12, [sp, #(STACKSIZE + 40)] // m1
	ldr		w13, [sp, #(STACKSIZE + 48)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_VS_LIB
#else
	CALL(inner_scale_ab_8x4_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 24)] // D
	ldr		w9, [sp, #(STACKSIZE + 32)] // ldd
	lsl		w9, w9, #3 // 8*sdd
	ldr		w10, [sp, #(STACKSIZE + 40)] // m1
	ldr		w11, [sp, #(STACKSIZE + 48)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB
#else
	CALL(inner_store_8x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dger2k_nt_8x4_vs_lib44cc)





//                                     w0     x1             x2          w3        x4          x5          w6        x7          sp+0          sp+8       sp+16    sp+24      sp+32
// void kernel_dsyr2k_nt_l_8x4_lib44cc(int k, double *alpha, double *A0, int sda0, double *B0, double *A1, int sda1, double *B1, double *beta, double *C, int ldc, double *D, int ldd);

	.align	4
	GLOB(kernel_dsyr2k_nt_l_8x4_lib44cc)
	FUN_START(kernel_dsyr2k_nt_l_8x4_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A0
	mov		w10, w3 // sda0
	lsl		w10, w10, #5 // 32*sda0
	mov		x11, x4 // B0

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif

	mov		w8, w0 // kmax
	mov		x9, x5 // A1
	mov		w10, w6 // sda1
	lsl		w10, w10, #5 // 32*sda1
	mov		x11, x7 // B1

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 24)] // D
	ldr		w9, [sp, #(STACKSIZE + 32)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_8X4_LIB
#else
	CALL(inner_prefetch_8x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	ldr		x9, [sp, #(STACKSIZE + 0)] // beta
	ldr		x10, [sp, #(STACKSIZE + 8)] // C
	ldr		w11, [sp, #(STACKSIZE + 16)] // ldc
	lsl		w11, w11, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB
#else
	CALL(inner_scale_ab_8x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 24)] // D
	ldr		w9, [sp, #(STACKSIZE + 32)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X4_LIB
#else
	CALL(inner_store_l_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dsyr2k_nt_l_8x4_lib44cc)





//                                   w0     x1             x2          w3        x4          x5          w6        x7          sp+0          sp+8       sp+16    sp+24      sp+32    sp+40   sp+48
// void kernel_dsyr2k_nt_l_8x4_lib44cc(int k, double *alpha, double *A0, int sda0, double *B0, double *A1, int sda1, double *B1, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.align	4
	GLOB(kernel_dsyr2k_nt_l_8x4_vs_lib44cc)
	FUN_START(kernel_dsyr2k_nt_l_8x4_vs_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A0
	mov		w10, w3 // sda0
	lsl		w10, w10, #5 // 32*sda0
	mov		x11, x4 // B0

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif

	mov		w8, w0 // kmax
	mov		x9, x5 // A1
	mov		w10, w6 // sda1
	lsl		w10, w10, #5 // 32*sda1
	mov		x11, x7 // B1

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	ldr		x9, [sp, #(STACKSIZE + 0)] // beta
	ldr		x10, [sp, #(STACKSIZE + 8)] // C
	ldr		w11, [sp, #(STACKSIZE + 16)] // ldc
	lsl		w11, w11, #3 // 8*sdc
	ldr		w12, [sp, #(STACKSIZE + 40)] // m1
	ldr		w13, [sp, #(STACKSIZE + 48)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_VS_LIB
#else
	CALL(inner_scale_ab_8x4_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 24)] // D
	ldr		w9, [sp, #(STACKSIZE + 32)] // ldd
	lsl		w9, w9, #3 // 8*sdd
	ldr		w10, [sp, #(STACKSIZE + 40)] // m1
	ldr		w11, [sp, #(STACKSIZE + 48)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X4_VS_LIB
#else
	CALL(inner_store_l_8x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dsyr2k_nt_l_8x4_vs_lib44cc)






//                                          w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8
// void kernel_dtrmm_nt_rl_4x8_tran_lib444c(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int ldd)

	.align	4
	GLOB(kernel_dtrmm_nt_rl_4x8_tran_lib444c)
	FUN_START(kernel_dtrmm_nt_rl_4x8_tran_lib444c)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NT_RL_8X4_LIB4
#else
	CALL(inner_edge_trmm_nt_rl_8x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // sdc
	lsl		w11, w11, #5 // 32*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB4
#else
	CALL(inner_scale_ab_8x4_lib4)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_TRAN_8X4_LIB
#else
	CALL(inner_tran_8x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_LIB
#else
	CALL(inner_store_4x8_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrmm_nt_rl_4x8_tran_lib444c)





//                                             w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8     sp+16   sp+24
// void kernel_dtrmm_nt_rl_4x8_tran_vs_lib444c(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dtrmm_nt_rl_4x8_tran_vs_lib444c)
	FUN_START(kernel_dtrmm_nt_rl_4x8_tran_vs_lib444c)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif

	ldr		w12, [sp, #(STACKSIZE + 16)] // m1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NT_RL_8X4_VS_LIB4
#else
	CALL(inner_edge_trmm_nt_rl_8x4_vs_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // sdc
	lsl		w11, w11, #5 // 32*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB4
#else
	CALL(inner_scale_ab_8x4_lib4)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*ldd
	ldr		w10, [sp, #(STACKSIZE + 16)] // m1
	ldr		w11, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_TRAN_8X4_LIB
#else
	CALL(inner_tran_8x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_VS_LIB
#else
	CALL(inner_store_4x8_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrmm_nt_rl_4x8_tran_vs_lib444c)





//                                          w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16
// void kernel_dtrmm_nt_rl_4x8_tran_lib4c4c(int kmax, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int ldd)

	.align	4
	GLOB(kernel_dtrmm_nt_rl_4x8_tran_lib4c4c)
	FUN_START(kernel_dtrmm_nt_rl_4x8_tran_lib4c4c)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // lda
	lsl		w10, w10, #5 // 32*lda
	mov		x11, x4 // B
	mov		w12, w5 // ldb
	lsl		w12, w12, #3 // 8*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4c)
#endif

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NT_RL_8X4_LIB4C
#else
	CALL(inner_edge_trmm_nt_rl_8x4_lib4c)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // ldc
	lsl		w11, w11, #5 // 32*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB4
#else
	CALL(inner_scale_ab_8x4_lib4)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_TRAN_8X4_LIB
#else
	CALL(inner_tran_8x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_LIB
#else
	CALL(inner_store_4x8_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrmm_nt_rl_4x8_tran_lib4c4c)





//                                             w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16    sp+24   sp+32
// void kernel_dtrmm_nt_rl_4x8_tran_vs_lib4c4c(int kmax, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dtrmm_nt_rl_4x8_tran_vs_lib4c4c)
	FUN_START(kernel_dtrmm_nt_rl_4x8_tran_vs_lib4c4c)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B
	mov		w12, w5 // ldb
	lsl		w12, w12, #3 // 8*ldb

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

	ldr		w13, [sp, #(STACKSIZE + 24)] // m1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NT_RL_8X4_VS_LIB4C
#else
	CALL(inner_edge_trmm_nt_rl_8x4_vs_lib4c)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // sdc
	lsl		w11, w11, #5 // 32*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB4
#else
	CALL(inner_scale_ab_8x4_lib4)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*ldd
	ldr		w10, [sp, #(STACKSIZE + 24)] // m1
	ldr		w11, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_TRAN_8X4_LIB
#else
	CALL(inner_tran_8x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_VS_LIB
#else
	CALL(inner_store_4x8_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrmm_nt_rl_4x8_tran_vs_lib4c4c)





//                                     w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8
// void kernel_dtrmm_nt_ru_8x4_lib444c(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int ldd)

	.align	4
	GLOB(kernel_dtrmm_nt_ru_8x4_lib444c)
	FUN_START(kernel_dtrmm_nt_ru_8x4_lib444c)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NT_RU_8X4_LIB4
#else
	CALL(inner_edge_trmm_nt_ru_8x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // sdc
	lsl		w11, w11, #5 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB4
#else
	CALL(inner_scale_ab_8x4_lib4)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrmm_nt_ru_8x4_lib444c)





//                                        w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8     sp+16   sp+14
// void kernel_dtrmm_nt_ru_8x4_vs_lib444c(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dtrmm_nt_ru_8x4_vs_lib444c)
	FUN_START(kernel_dtrmm_nt_ru_8x4_vs_lib444c)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NT_RU_8X4_VS_LIB4
#else
	CALL(inner_edge_trmm_nt_ru_8x4_vs_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // sdc
	lsl		w11, w11, #5 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB4
#else
	CALL(inner_scale_ab_8x4_lib4)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*ldd
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

	FUN_END(kernel_dtrmm_nt_ru_8x4_vs_lib444c)





//                                          w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8
// void kernel_dtrmm_nt_ru_4x8_tran_lib444c(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int ldd)

	.align	4
	GLOB(kernel_dtrmm_nt_ru_4x8_tran_lib444c)
	FUN_START(kernel_dtrmm_nt_ru_4x8_tran_lib444c)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NT_RU_8X4_LIB4
#else
	CALL(inner_edge_trmm_nt_ru_8x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // sdc
	lsl		w11, w11, #5 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB4
#else
	CALL(inner_scale_ab_8x4_lib4)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_TRAN_8X4_LIB
#else
	CALL(inner_tran_8x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_LIB
#else
	CALL(inner_store_4x8_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrmm_nt_ru_4x8_tran_lib444c)





//                                             w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8     sp+16   sp+14
// void kernel_dtrmm_nt_ru_4x8_tran_vs_lib444c(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dtrmm_nt_ru_4x8_tran_vs_lib444c)
	FUN_START(kernel_dtrmm_nt_ru_4x8_tran_vs_lib444c)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NT_RU_8X4_VS_LIB4
#else
	CALL(inner_edge_trmm_nt_ru_8x4_vs_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // sdc
	lsl		w11, w11, #5 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB4
#else
	CALL(inner_scale_ab_8x4_lib4)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*ldd
	ldr		w10, [sp, #(STACKSIZE + 16)] // m1
	ldr		w11, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_TRAN_8X4_LIB
#else
	CALL(inner_tran_8x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_VS_LIB
#else
	CALL(inner_store_4x8_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrmm_nt_ru_4x8_tran_vs_lib444c)





//                                     w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16
// void kernel_dtrmm_nn_rl_8x4_lib4ccc(int kmax, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dtrmm_nn_rl_8x4_lib4ccc)
	FUN_START(kernel_dtrmm_nn_rl_8x4_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // ldb
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B
	mov		w12, w5 // ldb
	lsl		w12, w12, #3 // 8*ldb

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NN_RL_8X4_LIB4C
#else
	CALL(inner_edge_trmm_nn_rl_8x4_lib4c)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_8x4_lib4c)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w11, w11, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB
#else
	CALL(inner_scale_ab_8x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrmm_nn_rl_8x4_lib4ccc)





//                                        w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16    sp+24   sp+32
// void kernel_dtrmm_nn_rl_8x4_vs_lib4ccc(int kmax, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dtrmm_nn_rl_8x4_vs_lib4ccc)
	FUN_START(kernel_dtrmm_nn_rl_8x4_vs_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // ldb
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B
	mov		w12, w5 // ldb
	lsl		w12, w12, #3 // 8*ldb

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NN_RL_8X4_VS_LIB4C
#else
	CALL(inner_edge_trmm_nn_rl_8x4_vs_lib4c)
#endif

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



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w11, w11, #3 // 8*ldd
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
	lsl		w9, w9, #3 // 8*ldd
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

	FUN_END(kernel_dtrmm_nn_rl_8x4_vs_lib4ccc)





//                                          w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16
// void kernel_dtrmm_nn_rl_4x8_tran_lib4c4c(int kmax, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int ldd)

	.align	4
	GLOB(kernel_dtrmm_nn_rl_4x8_tran_lib4c4c)
	FUN_START(kernel_dtrmm_nn_rl_4x8_tran_lib4c4c)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // ldb
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B
	mov		w12, w5 // ldb
	lsl		w12, w12, #3 // 8*ldb

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NN_RL_8X4_LIB4C
#else
	CALL(inner_edge_trmm_nn_rl_8x4_lib4c)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_8x4_lib4c)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w11, w11, #5 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB4
#else
	CALL(inner_scale_ab_8x4_lib4)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_TRAN_8X4_LIB
#else
	CALL(inner_tran_8x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_LIB
#else
	CALL(inner_store_4x8_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrmm_nn_rl_4x8_tran_lib4c4c)





//                                             w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16    sp+24   sp+32
// void kernel_dtrmm_nn_rl_4x8_tran_vs_lib4c4c(int kmax, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dtrmm_nn_rl_4x8_tran_vs_lib4c4c)
	FUN_START(kernel_dtrmm_nn_rl_4x8_tran_vs_lib4c4c)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // ldb
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B
	mov		w12, w5 // ldb
	lsl		w12, w12, #3 // 8*ldb

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NN_RL_8X4_VS_LIB4C
#else
	CALL(inner_edge_trmm_nn_rl_8x4_vs_lib4c)
#endif

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



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w11, w11, #5 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB4
#else
	CALL(inner_scale_ab_8x4_lib4)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*ldd
	ldr		w10, [sp, #(STACKSIZE + 24)] // m1
	ldr		w11, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_TRAN_8X4_LIB
#else
	CALL(inner_tran_8x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_VS_LIB
#else
	CALL(inner_store_4x8_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrmm_nn_rl_4x8_tran_vs_lib4c4c)





//                                         w0        x1         w2       x3         x4         w5       x6         w7       sp+0       sp+8     sp+16
// void kernel_dtrsm_nt_rl_inv_8x4_lib44ccc(int kmax, double *A, int sda, double *B, double *C, int ldc, double *D, int ldd, double *E, int lde, double *inv_diag_E)

	.align	4
	GLOB(kernel_dtrsm_nt_rl_inv_8x4_lib44ccc)
	FUN_START(kernel_dtrsm_nt_rl_inv_8x4_lib44ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x4 // C
	mov		w9, w5 // ldc
	lsl		w9, w9, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8X4_LIB
#else
	CALL(inner_scale_m11_8x4_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 0)] // E
	ldr		w9, [sp, #(STACKSIZE + 8)] // sde
	lsl		w9, w9, #3 // 8*ldc
	ldr		x10, [sp, #(STACKSIZE + 16)] // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_8X4_LIB
#else
	CALL(inner_edge_trsm_rlt_inv_8x4_lib)
#endif



	// store l
	mov		x8, x6 // D
	mov		w9, w7 // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_8x4_lib44ccc)





//                                            w0        x1         w2       x3         x4         w5       x6         w7       sp+0       sp+8     sp+16               sp+24   sp+32
// void kernel_dtrsm_nt_rl_inv_8x4_vs_lib44ccc(int kmax, double *A, int sda, double *B, double *C, int ldc, double *D, int ldd, double *E, int lde, double *inv_diag_E, int m1, int n1)

	.align	4
	GLOB(kernel_dtrsm_nt_rl_inv_8x4_vs_lib44ccc)
	FUN_START(kernel_dtrsm_nt_rl_inv_8x4_vs_lib44ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x4 // C
	mov		w9, w5 // ldc
	lsl		w9, w9, #3 // 8*ldc
	ldr		w10, [sp, #(STACKSIZE + 24)] // m1
	ldr		w11, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8X4_VS_LIB
#else
	CALL(inner_scale_m11_8x4_vs_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 0)] // E
	ldr		w9, [sp, #(STACKSIZE + 8)] // sde
	lsl		w9, w9, #3 // 8*ldc
	ldr		x10, [sp, #(STACKSIZE + 16)] // inv_diag_E
	ldr		w11, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_8X4_VS_LIB
#else
	CALL(inner_edge_trsm_rlt_inv_8x4_vs_lib)
#endif



	// store l
	mov		x8, x6 // D
	mov		w9, w7 // ldd
	lsl		w9, w9, #3 // 8*ldd
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

	FUN_END(kernel_dtrsm_nt_rl_inv_8x4_vs_lib44ccc)





//                                          w0        x1         w2       x3         x4            x5         w6       x7         sp+0     sp+8       sp+16
// void kernel_dtrsm_nt_rl_inv_8x4_lib44cc4(int kmax, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E, double *inv_diag_E)

	.align	4
	GLOB(kernel_dtrsm_nt_rl_inv_8x4_lib44cc4)
	FUN_START(kernel_dtrsm_nt_rl_inv_8x4_lib44cc4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta
	mov		x8, x4 // beta
	mov		x9, x5 // C
	mov		w10, w6 // ldc
	lsl		w10, w10, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_8X4_LIB
#else
	CALL(inner_scale_m1b_8x4_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 8)] // E
	ldr		x9, [sp, #(STACKSIZE + 16)] // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_8X4_LIB4
#else
	CALL(inner_edge_trsm_rlt_inv_8x4_lib4)
#endif



	// store l
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_8x4_lib44cc4)





//                                             w0        x1         w2       x3         x4            x5         w6       x7         sp+0     sp+8       sp+16               sp+24   sp+32
// void kernel_dtrsm_nt_rl_inv_8x4_vs_lib44cc4(int kmax, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E, double *inv_diag_E, int m1, int n1)

	.align	4
	GLOB(kernel_dtrsm_nt_rl_inv_8x4_vs_lib44cc4)
	FUN_START(kernel_dtrsm_nt_rl_inv_8x4_vs_lib44cc4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
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
	lsl		w10, w10, #3 // 8*ldc
	ldr		w11, [sp, #(STACKSIZE + 24)] // m1
	ldr		w12, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_8X4_VS_LIB
#else
	CALL(inner_scale_m1b_8x4_vs_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 8)] // E
	ldr		x9, [sp, #(STACKSIZE + 16)] // inv_diag_E
	ldr		w10, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_8X4_VS_LIB4
#else
	CALL(inner_edge_trsm_rlt_inv_8x4_vs_lib4)
#endif



	// store l
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w9, w9, #3 // 8*ldd
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

	FUN_END(kernel_dtrsm_nt_rl_inv_8x4_vs_lib44cc4)





//                                          w0        x1         w2       x3         w4       x5            x6         w7       sp+0       sp+8     sp+16      sp+24    sp+32
// void kernel_dtrsm_nt_rl_inv_8x4_lib4cccc(int kmax, double *A, int sda, double *B, int ldb, dobule *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, double *inv_diag_E)

	.align	4
	GLOB(kernel_dtrsm_nt_rl_inv_8x4_lib4cccc)
	FUN_START(kernel_dtrsm_nt_rl_inv_8x4_lib4cccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x3 // B
	mov		w12, w4 // sda
	lsl		w12, w12, #3 // 8*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4c)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x5 // beta
	mov		x9, x6 // C
	mov		w10, w7 // ldc
	lsl		w10, w10, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_8X4_LIB
#else
	CALL(inner_scale_m1b_8x4_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 16)] // E
	ldr		w9, [sp, #(STACKSIZE + 24)] // sde
	lsl		w9, w9, #3 // 8*ldc
	ldr		x10, [sp, #(STACKSIZE + 32)] // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_8X4_LIB
#else
	CALL(inner_edge_trsm_rlt_inv_8x4_lib)
#endif



	// store l
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_8x4_lib4cccc)





//                                             w0        x1         w2       x3         w4       x5            x6         w7       sp+0       sp+8     sp+16      sp+24    sp+32               sp+40   sp+48
// void kernel_dtrsm_nt_rl_inv_8x4_vs_lib4cccc(int kmax, double *A, int sda, double *B, int ldb, dobule *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, double *inv_diag_E, int m1, int n1)

	.align	4
	GLOB(kernel_dtrsm_nt_rl_inv_8x4_vs_lib4cccc)
	FUN_START(kernel_dtrsm_nt_rl_inv_8x4_vs_lib4cccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x3 // B
	mov		w12, w4 // sda
	lsl		w12, w12, #3 // 8*ldb

	ldr		w13, [sp, #(STACKSIZE + 48)] // n1
	cmp		w13, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X1_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_8x1_lib4c)
#endif

	b		103f

100:

	ldr		w13, [sp, #(STACKSIZE + 48)] // n1
	cmp		w13, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X2_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_8x2_lib4c)
#endif
	
	b		103f

101:

	ldr		w13, [sp, #(STACKSIZE + 48)] // n1
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



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x5 // beta
	mov		x9, x6 // C
	mov		w10, w7 // ldc
	lsl		w10, w10, #3 // 8*ldc
	ldr		w11, [sp, #(STACKSIZE + 40)] // m1
	ldr		w12, [sp, #(STACKSIZE + 48)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_8X4_VS_LIB
#else
	CALL(inner_scale_m1b_8x4_vs_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 16)] // E
	ldr		w9, [sp, #(STACKSIZE + 24)] // sde
	lsl		w9, w9, #3 // 8*ldc
	ldr		x10, [sp, #(STACKSIZE + 32)] // inv_diag_E
	ldr		w11, [sp, #(STACKSIZE + 48)] // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_8X4_VS_LIB
#else
	CALL(inner_edge_trsm_rlt_inv_8x4_vs_lib)
#endif



	// store l
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*ldd
	ldr		w10, [sp, #(STACKSIZE + 40)] // m1
	ldr		w11, [sp, #(STACKSIZE + 48)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB
#else
	CALL(inner_store_8x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_8x4_vs_lib4cccc)





//                                         w0        x1         w2       x3         x4            x5         w6       x7         sp+0     sp+8
// void kernel_dtrsm_nt_rl_one_8x4_lib44cc4(int kmax, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E)

	.align	4
	GLOB(kernel_dtrsm_nt_rl_one_8x4_lib44cc4)
	FUN_START(kernel_dtrsm_nt_rl_one_8x4_lib44cc4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
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
	lsl		w10, w10, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_8X4_LIB
#else
	CALL(inner_scale_m1b_8x4_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 8)] // E

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_ONE_8X4_LIB4
#else
	CALL(inner_edge_trsm_rlt_one_8x4_lib4)
#endif



	// store l
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nt_rl_one_8x4_lib44cc4)





//                                         w0        x1         w2       x3         x4            x5         w6       x7         sp+0     sp+8       sp+16      sp+24
// void kernel_dtrsm_nt_rl_one_8x4_vs_lib44cc4(int kmax, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E, int m1, int n1)

	.align	4
	GLOB(kernel_dtrsm_nt_rl_one_8x4_vs_lib44cc4)
	FUN_START(kernel_dtrsm_nt_rl_one_8x4_vs_lib44cc4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
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
	lsl		w10, w10, #3 // 8*ldc
	ldr		w11, [sp, #(STACKSIZE + 16)] // m1
	ldr		w12, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_8X4_VS_LIB
#else
	CALL(inner_scale_m1b_8x4_vs_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 8)] // E
	ldr		w9, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_ONE_8X4_VS_LIB4
#else
	CALL(inner_edge_trsm_rlt_one_8x4_vs_lib4)
#endif



	// store l
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w9, w9, #3 // 8*ldd
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

	FUN_END(kernel_dtrsm_nt_rl_one_8x4_vs_lib44cc4)





//                                          w0        x1         w2       x3         x4            x5         w6       x7         sp+0     sp+8       sp+16
// void kernel_dtrsm_nt_ru_inv_8x4_lib44cc4(int kmax, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E, double *inv_diag_E)

	.align	4
	GLOB(kernel_dtrsm_nt_ru_inv_8x4_lib44cc4)
	FUN_START(kernel_dtrsm_nt_ru_inv_8x4_lib44cc4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta
	mov		x8, x4 // beta
	mov		x9, x5 // C
	mov		w10, w6 // ldc
	lsl		w10, w10, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_8X4_LIB
#else
	CALL(inner_scale_m1b_8x4_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 8)] // E
	ldr		x9, [sp, #(STACKSIZE + 16)] // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RUT_INV_8X4_LIB4
#else
	CALL(inner_edge_trsm_rut_inv_8x4_lib4)
#endif



	// store l
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nt_ru_inv_8x4_lib44cc4)





//                                             w0        x1         w2       x3         x4            x5         w6       x7         sp+0     sp+8       sp+16               sp+24   sp+32
// void kernel_dtrsm_nt_ru_inv_8x4_vs_lib44cc4(int kmax, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E, double *inv_diag_E, int m1, int n1)

	.align	4
	GLOB(kernel_dtrsm_nt_ru_inv_8x4_vs_lib44cc4)
	FUN_START(kernel_dtrsm_nt_ru_inv_8x4_vs_lib44cc4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
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
	lsl		w10, w10, #3 // 8*ldc
	ldr		w11, [sp, #(STACKSIZE + 24)] // m1
	ldr		w12, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_8X4_VS_LIB
#else
	CALL(inner_scale_m1b_8x4_vs_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 8)] // E
	ldr		x9, [sp, #(STACKSIZE + 16)] // inv_diag_E
	ldr		w10, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RUT_INV_8X4_VS_LIB4
#else
	CALL(inner_edge_trsm_rut_inv_8x4_vs_lib4)
#endif



	// store l
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w9, w9, #3 // 8*ldd
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

	FUN_END(kernel_dtrsm_nt_ru_inv_8x4_vs_lib44cc4)





//                                    w0        x1         w2       x3         x4         w5       x6         w7       sp+0
// void kernel_dpotrf_nt_l_8x4_lib44cc(int kmax, double *A, int sda, double *B, double *C, int ldc, double *D, int ldd, double *inv_diag_D)

	.align	4
	GLOB(kernel_dpotrf_nt_l_8x4_lib44cc)
	FUN_START(kernel_dpotrf_nt_l_8x4_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x4 // C
	mov		w9, w5 // ldc
	lsl		w9, w9, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8X4_LIB
#else
	CALL(inner_scale_m11_8x4_lib)
#endif



	// factorization
	ldr		x8, [sp, #(STACKSIZE + 0)] // inv_diag_D

#if MACRO_LEVEL>=1
	INNER_EDGE_POTRF_8X4_LIB4
#else
	CALL(inner_edge_potrf_8x4_lib4)
#endif



	// store l
	mov		x8, x6 // D
	mov		w9, w7 // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X4_LIB
#else
	CALL(inner_store_l_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dpotrf_nt_l_8x4_lib44cc)





//                                       w0        x1         w2       x3         x4         xw       x6         w7       sp+0                sp+8,   sp+16
// void kernel_dpotrf_nt_l_8x4_vs_lib44cc(int kmax, double *A, int sda, double *B, double *C, int ldc, double *D, int ldd, double *inv_diag_D, int m1, int n1)

	.align	4
	GLOB(kernel_dpotrf_nt_l_8x4_vs_lib44cc)
	FUN_START(kernel_dpotrf_nt_l_8x4_vs_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x4 // C
	mov		w9, w5 // ldc
	lsl		w9, w9, #3 // 8*ldc
	ldr		w10, [sp, #(STACKSIZE + 8)] // m1
	ldr		w11, [sp, #(STACKSIZE + 16)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8X4_VS_LIB
#else
	CALL(inner_scale_m11_8x4_vs_lib)
#endif



	// factorization
	ldr		x8, [sp, #(STACKSIZE + 0)] // inv_diag_D
	ldr		w9, [sp, #(STACKSIZE + 16)] // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_POTRF_8X4_VS_LIB4
#else
	CALL(inner_edge_potrf_8x4_vs_lib4)
#endif



	// store l
	mov		x8, x6 // D
	mov		w9, w7 // ldd
	lsl		w9, w9, #3 // 8*ldd
	ldr		w10, [sp, #(STACKSIZE + 8)] // m1
	ldr		w11, [sp, #(STACKSIZE + 16)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X4_VS_LIB
#else
	CALL(inner_store_l_8x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dpotrf_nt_l_8x4_vs_lib44cc)





//                                         w0        x1         w2       x3         w4       x5            x6         w7       sp+0       sp+8     sp+16      sp+24
// void kernel_dtrsm_nn_ll_one_8x4_lib4cccc(int kmax, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde)

	.align	4
	GLOB(kernel_dtrsm_nn_ll_one_8x4_lib4cccc)
	FUN_START(kernel_dtrsm_nn_ll_one_8x4_lib4cccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // ldb
	lsl		w10, w10, #5 // 32*ldb
	mov		x11, x3 // B
	mov		w12, w4 // ldb
	lsl		w12, w12, #3 // 8*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_8x4_lib4c)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x5 // beta
	mov		x9, x6 // C
	mov		w10, w7 // ldc
	lsl		w10, w10, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_8X4_LIB
#else
	CALL(inner_scale_m1b_8x4_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 16)] // E
	ldr		w9, [sp, #(STACKSIZE + 24)] // sde
	lsl		w9, w9, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_LLN_ONE_8X4_LIBC
#else
	CALL(inner_edge_trsm_lln_one_8x4_libc)
#endif



	// store l
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // sdd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nn_ll_one_8x4_lib4cccc)







//                                            w0        x1         w2       x3         w4       x5            x6         w7       sp+0       sp+8     sp+16      sp+24    sp+32   sp+40
// void kernel_dtrsm_nn_ll_one_8x4_vs_lib4cccc(int kmax, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, int m1, int n1)

	.align	4
	GLOB(kernel_dtrsm_nn_ll_one_8x4_vs_lib4cccc)
	FUN_START(kernel_dtrsm_nn_ll_one_8x4_vs_lib4cccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x3 // B
	mov		w12, w4 // ldb
	lsl		w12, w12, #3 // 8*ldb

	ldr		w13, [sp, #(STACKSIZE + 40)] // n1
	cmp		w13, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X1_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_8x1_lib4c)
#endif

	b		103f

100:

	ldr		w13, [sp, #(STACKSIZE + 40)] // n1
	cmp		w13, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X2_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_8x2_lib4c)
#endif
	
	b		103f

101:

	ldr		w13, [sp, #(STACKSIZE + 40)] // n1
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



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x5 // beta
	mov		x9, x6 // C
	mov		w10, w7 // ldc
	lsl		w10, w10, #3 // 8*ldc
	ldr		w11, [sp, #(STACKSIZE + 32)] // m1
	ldr		w12, [sp, #(STACKSIZE + 40)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_8X4_VS_LIB
#else
	CALL(inner_scale_m1b_8x4_vs_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 16)] // E
	ldr		w9, [sp, #(STACKSIZE + 24)] // sde
	lsl		w9, w9, #3 // 8*ldc
	ldr		w10, [sp, #(STACKSIZE + 32)] // m1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_LLN_ONE_8X4_VS_LIBC
#else
	CALL(inner_edge_trsm_lln_one_8x4_vs_libc)
#endif



	// store l
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // sdd
	lsl		w9, w9, #3 // 8*ldd
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

	FUN_END(kernel_dtrsm_nn_ll_one_8x4_vs_lib4cccc)






//                                          w0        x1         w2       x3         w4       x5            x6         w7       sp+0       sp+8     sp+16      sp+24    sp+32
// void kernel_dtrsm_nn_rl_inv_8x4_lib4cccc(int kmax, double *A, int sda, double *B, int ldb, dobule *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, double *inv_diag_E)

	.align	4
	GLOB(kernel_dtrsm_nn_rl_inv_8x4_lib4cccc)
	FUN_START(kernel_dtrsm_nn_rl_inv_8x4_lib4cccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x3 // B
	mov		w12, w4 // sda
	lsl		w12, w12, #3 // 8*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_8x4_lib4c)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x5 // beta
	mov		x9, x6 // C
	mov		w10, w7 // ldc
	lsl		w10, w10, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_8X4_LIB
#else
	CALL(inner_scale_m1b_8x4_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 16)] // E
	ldr		w9, [sp, #(STACKSIZE + 24)] // sde
	lsl		w9, w9, #3 // 8*ldc
	ldr		x10, [sp, #(STACKSIZE + 32)] // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLN_INV_8X4_LIB
#else
	CALL(inner_edge_trsm_rln_inv_8x4_lib)
#endif



	// store l
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nn_rl_inv_8x4_lib4cccc)





//                                             w0        x1         w2       x3         w4       x5            x6         w7       sp+0       sp+8     sp+16      sp+24    sp+32               sp+40   sp+48
// void kernel_dtrsm_nn_rl_inv_8x4_vs_lib4cccc(int kmax, double *A, int sda, double *B, int ldb, dobule *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, double *inv_diag_E, int m1, int n1)

	.align	4
	GLOB(kernel_dtrsm_nn_rl_inv_8x4_vs_lib4cccc)
	FUN_START(kernel_dtrsm_nn_rl_inv_8x4_vs_lib4cccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x3 // B
	mov		w12, w4 // sda
	lsl		w12, w12, #3 // 8*ldb

	ldr		w13, [sp, #(STACKSIZE + 48)] // n1
	cmp		w13, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X1_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_8x1_lib4c)
#endif

	b		103f

100:

	ldr		w13, [sp, #(STACKSIZE + 48)] // n1
	cmp		w13, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X2_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_8x2_lib4c)
#endif
	
	b		103f

101:

	ldr		w13, [sp, #(STACKSIZE + 48)] // n1
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



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x5 // beta
	mov		x9, x6 // C
	mov		w10, w7 // ldc
	lsl		w10, w10, #3 // 8*ldc
	ldr		w11, [sp, #(STACKSIZE + 40)] // m1
	ldr		w12, [sp, #(STACKSIZE + 48)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_8X4_VS_LIB
#else
	CALL(inner_scale_m1b_8x4_vs_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 16)] // E
	ldr		w9, [sp, #(STACKSIZE + 24)] // sde
	lsl		w9, w9, #3 // 8*ldc
	ldr		x10, [sp, #(STACKSIZE + 32)] // inv_diag_E
	ldr		w11, [sp, #(STACKSIZE + 48)] // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLN_INV_8X4_VS_LIB
#else
	CALL(inner_edge_trsm_rln_inv_8x4_vs_lib)
#endif



	// store l
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*ldd
	ldr		w10, [sp, #(STACKSIZE + 40)] // m1
	ldr		w11, [sp, #(STACKSIZE + 48)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB
#else
	CALL(inner_store_8x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nn_rl_inv_8x4_vs_lib4cccc)





//                                          w0        x1         w2       x3         w4       x5            x6         w7       sp+0       sp+8     sp+16      sp+24    sp+32
// void kernel_dtrsm_nn_ru_inv_8x4_lib4c44c(int kmax, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int sdd, double *E, int lde, double *inv_diag_E)

	.align	4
	GLOB(kernel_dtrsm_nn_ru_inv_8x4_lib4c44c)
	FUN_START(kernel_dtrsm_nn_ru_inv_8x4_lib4c44c)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x3 // B
	mov		w12, w4 // ldb
	lsl		w12, w12, #3 // 8*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_8x4_lib4c)
#endif



	// call inner blend for alpha=1.0 and beta
	mov		x8, x5 // beta
	mov		x9, x6 // C
	mov		w10, w7 // ldc
	lsl		w10, w10, #5 // 32*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_8X4_LIB4
#else
	CALL(inner_scale_m1b_8x4_lib4)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 16)] // E
	ldr		w9, [sp, #(STACKSIZE + 24)] // sde
	lsl		w9, w9, #3 // 8*ldc
	ldr		x10, [sp, #(STACKSIZE + 32)] // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RUN_INV_8X4_LIB
#else
	CALL(inner_edge_trsm_run_inv_8x4_lib)
#endif



	// store
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #5 // 32*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB4
#else
	CALL(inner_store_8x4_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nn_ru_inv_8x4_lib4c44c)






//                                             w0        x1         w2       x3         w4       x5            x6         w7       sp+0       sp+8     sp+16      sp+24    sp+32               sp+40   sp+48
// void kernel_dtrsm_nn_ru_inv_8x4_vs_lib4c44c(int kmax, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int sdd, double *E, int lde, double *inv_diag_E, int m1, int n1)

	.align	4
	GLOB(kernel_dtrsm_nn_ru_inv_8x4_vs_lib4c44c)
	FUN_START(kernel_dtrsm_nn_ru_inv_8x4_vs_lib4c44c)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x3 // B
	mov		w12, w4 // ldb
	lsl		w12, w12, #3 // 8*ldb

	ldr		w13, [sp, #(STACKSIZE + 48)] // n1
	cmp		w13, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X1_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_8x1_lib4c)
#endif

	b		103f

100:

	ldr		w13, [sp, #(STACKSIZE + 48)] // n1
	cmp		w13, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X2_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_8x2_lib4c)
#endif
	
	b		103f

101:

	ldr		w13, [sp, #(STACKSIZE + 48)] // n1
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



	// call inner blend for alpha=1.0 and beta
	mov		x8, x5 // beta
	mov		x9, x6 // C
	mov		w10, w7 // ldc
	lsl		w10, w10, #5 // 32*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_8X4_LIB4
#else
	CALL(inner_scale_m1b_8x4_lib4)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 16)] // E
	ldr		w9, [sp, #(STACKSIZE + 24)] // sde
	lsl		w9, w9, #3 // 8*ldc
	ldr		x10, [sp, #(STACKSIZE + 32)] // inv_diag_E
	ldr		w11, [sp, #(STACKSIZE + 48)] // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RUN_INV_8X4_VS_LIB
#else
	CALL(inner_edge_trsm_run_inv_8x4_vs_lib)
#endif



	// store
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #5 // 32*sdd
	ldr		w10, [sp, #(STACKSIZE + 40)] // m1
	ldr		w11, [sp, #(STACKSIZE + 48)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB4
#else
	CALL(inner_store_8x4_vs_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nn_ru_inv_8x4_vs_lib4c44c)





//                                 w0        x1             x2         x3       xw         x5            x6         w7       sp+0       sp+8
// void kernel_dgemm_nt_4x8_lib44cc(int kmax, double *alpha, double *A, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dgemm_nt_4x8_lib44cc)
	FUN_START(kernel_dgemm_nt_4x8_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// prefetch C
//	mov		x8, x5 // beta
	ld1		{v29.2d}, [x5]
	fcmpe	d29, #0.0
	beq		100f

	mov		x8, x6 // C
	mov		w9, w7 // ldc
	lsl		w9, w9, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_PREFETCH_4X8_LIB
#else
	CALL(inner_prefetch_4x8_lib)
#endif

100:


#if 1

	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B
	mov		w11, w4 // sdb
	lsl		w11, w11, #5 // 32*sdb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X8_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_4x8_lib4)
#endif

#else

	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x3 // B
	mov		w10, w4 // sdb
	lsl		w10, w10, #5 // 32*sdb
	mov		x11, x2 // A

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_8X4_LIB
#else
	CALL(inner_tran_8x4_lib)
#endif

#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_4X8_LIB
#else
	CALL(inner_prefetch_4x8_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // ldc
	lsl		w11, w11, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X8_LIB
#else
	CALL(inner_scale_ab_4x8_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_LIB
#else
	CALL(inner_store_4x8_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nt_4x8_lib44cc)





//                                    w0        x1             x2         x3       w4         x5            x6         w7       sp+0       sp+8     sp+16   sp+24
// void kernel_dgemm_nt_4x8_vs_lib44cc(int kmax, double *alpha, double *A, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dgemm_nt_4x8_vs_lib44cc)
	FUN_START(kernel_dgemm_nt_4x8_vs_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x3 // B
	mov		w10, w4 // sdb
	lsl		w10, w10, #5 // 32*sdb
	mov		x11, x2 // A

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_8X4_LIB
#else
	CALL(inner_tran_8x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // ldc
	lsl		w11, w11, #3 // 8*sdc
	ldr		w12, [sp, #(STACKSIZE + 16)] // m1
	ldr		w13, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X8_VS_LIB
#else
	CALL(inner_scale_ab_4x8_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd
	ldr		w10, [sp, #(STACKSIZE + 16)] // m1
	ldr		w11, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_VS_LIB
#else
	CALL(inner_store_4x8_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nt_4x8_vs_lib44cc)






//                                 w0        x1             x2         x3         w4       x5            x6         w7       sp+0       sp+8
// void kernel_dgemm_nt_4x8_lib4ccc(int kmax, double *alpha, double *A, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dgemm_nt_4x8_lib4ccc)
	FUN_START(kernel_dgemm_nt_4x8_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B
	mov		w11, w4 // ldb
	lsl		w11, w11, #3 // 8*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X8_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_4x8_lib4c)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_4X8_LIB
#else
	CALL(inner_prefetch_4x8_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // ldc
	lsl		w11, w11, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X8_LIB
#else
	CALL(inner_scale_ab_4x8_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_LIB
#else
	CALL(inner_store_4x8_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nt_4x8_lib4ccc)






//                                     w0        x1             x2         x3         w4       x5            x6         w7       sp+0       sp+8     sp+16   sp+24
// void kernel_dgemm_nt_4x8_vs_lib4ccc(int kmax, double *alpha, double *A, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dgemm_nt_4x8_vs_lib4ccc)
	FUN_START(kernel_dgemm_nt_4x8_vs_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B
	mov		w11, w4 // ldb
	lsl		w11, w11, #3 // 8*ldb

	ldr		w13, [sp, #(STACKSIZE + 24)] // n1
	cmp		w13, #5
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X5_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_4x5_lib4c)
#endif

	b		103f

100:

	ldr		w13, [sp, #(STACKSIZE + 24)] // n1
	cmp		w13, #6
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X6_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_4x6_lib4c)
#endif
	
	b		103f

101:

	ldr		w13, [sp, #(STACKSIZE + 24)] // n1
	cmp		w13, #7
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X7_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_4x7_lib4c)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X8_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_4x8_lib4c)
#endif

103:



	// prefetch
//	ldr		x8, [sp, #(STACKSIZE + 0)] // D
//	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
//	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
//	INNER_PREFETCH_4X8_LIB
#else
//	CALL(inner_prefetch_4x8_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // ldc
	lsl		w11, w11, #3 // 8*sdc
	ldr		w12, [sp, #(STACKSIZE + 16)] // m1
	ldr		w13, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X8_VS_LIB
#else
	CALL(inner_scale_ab_4x8_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd
	ldr		w10, [sp, #(STACKSIZE + 16)] // m1
	ldr		w11, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_VS_LIB
#else
	CALL(inner_store_4x8_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nt_4x8_vs_lib4ccc)






//                                  w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16
// void kernel_dgemm_nt_4x8_libc4cc(int kmax, double *alpha, double *A, int lda, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dgemm_nt_4x8_libc4cc)
	FUN_START(kernel_dgemm_nt_4x8_libc4cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x4 // B
	mov		w10, w5 // sdb
	lsl		w10, w10, #5 // 32*sdb
	mov		x11, x2 // A
	mov		w12, w3 // lda
	lsl		w12, w12, #3 // 8*lda

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4c)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*sdc

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
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_LIB
#else
	CALL(inner_store_4x8_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nt_4x8_libc4cc)






//                                     w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16    sp+24   sp+32
// void kernel_dgemm_nt_4x8_vs_libc4cc(int kmax, double *alpha, double *A, int lda, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dgemm_nt_4x8_vs_libc4cc)
	FUN_START(kernel_dgemm_nt_4x8_vs_libc4cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x4 // B
	mov		w10, w5 // sdb
	lsl		w10, w10, #5 // 32*sdb
	mov		x11, x2 // A
	mov		w12, w3 // lda
	lsl		w12, w12, #3 // 8*lda

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
	// TODO prefetch vs
//	ldr		x8, [sp, #(STACKSIZE + 8)] // D
//	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
//	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*sdc
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
	lsl		w9, w9, #3 // 8*sdd
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

	FUN_END(kernel_dgemm_nt_4x8_vs_libc4cc)






//                                 w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16
// void kernel_dgemm_nt_4x8_libcccc(int kmax, double *alpha, double *A, int lda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dgemm_nt_4x8_libcccc)
	FUN_START(kernel_dgemm_nt_4x8_libcccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x4 // B
	mov		w10, w5 // sdb
	lsl		w10, w10, #3 // 8*sdb
	mov		x11, x2 // A
	mov		w12, w3 // lda
	lsl		w12, w12, #3 // 8*lda

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIBCC
#else
	CALL(inner_kernel_gemm_add_nt_8x4_libcc)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*sdc

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
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_LIB
#else
	CALL(inner_store_4x8_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nt_4x8_libcccc)






//                                     w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16    sp+24   sp+32
// void kernel_dgemm_nt_4x8_vs_libcccc(int kmax, double *alpha, double *A, int lda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dgemm_nt_4x8_vs_libcccc)
	FUN_START(kernel_dgemm_nt_4x8_vs_libcccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x4 // B
	mov		w10, w5 // sdb
	lsl		w10, w10, #3 // 8*sdb
	mov		x11, x2 // A
	mov		w12, w3 // lda
	lsl		w12, w12, #3 // 8*lda
	ldr		w13, [sp, #(STACKSIZE + 32)] // n1

	ldr		w14, [sp, #(STACKSIZE + 24)] // m1

	cmp		w13, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X1_VS_LIBCC
#else
	CALL(inner_kernel_gemm_add_nt_8x1_vs_libcc)
#endif

	b		103f

100:

	cmp		w14, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X2_VS_LIBCC
#else
	CALL(inner_kernel_gemm_add_nt_8x2_vs_libcc)
#endif
	
	b		103f

101:

	cmp		w14, #3
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X3_VS_LIBCC
#else
	CALL(inner_kernel_gemm_add_nt_8x3_vs_libcc)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_VS_LIBCC
#else
	CALL(inner_kernel_gemm_add_nt_8x4_vs_libcc)
#endif

103:



	// prefetch
//	ldr		x8, [sp, #(STACKSIZE + 8)] // D
//	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
//	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*sdc
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
	lsl		w9, w9, #3 // 8*sdd
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

	FUN_END(kernel_dgemm_nt_4x8_vs_libcccc)






//                                 w0        x1             x2         x3         w4       x5            x6         w7       sp+0       sp+8
// void kernel_dgemm_nn_4x8_lib4ccc(int kmax, double *alpha, double *A, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dgemm_nn_4x8_lib4ccc)
	FUN_START(kernel_dgemm_nn_4x8_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B
	mov		w11, w4 // sdb
	lsl		w11, w11, #3 // 8*sdb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X8_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_4x8_lib4c)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_4X8_LIB
#else
	CALL(inner_prefetch_4x8_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // ldc
	lsl		w11, w11, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X8_LIB
#else
	CALL(inner_scale_ab_4x8_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_LIB
#else
	CALL(inner_store_4x8_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nn_4x8_lib4ccc)






//                                     w0        x1             x2         x3         w4       x5            x6         w7       sp+0       sp+8     sp+16   sp+24
// void kernel_dgemm_nn_4x8_vs_lib4ccc(int kmax, double *alpha, double *A, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dgemm_nn_4x8_vs_lib4ccc)
	FUN_START(kernel_dgemm_nn_4x8_vs_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B
	mov		w11, w4 // sdb
	lsl		w11, w11, #3 // 8*sdb

	ldr		w13, [sp, #(STACKSIZE + 24)] // n1
	cmp		w13, #5
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X5_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_4x5_lib4c)
#endif

	b		103f

100:

	ldr		w13, [sp, #(STACKSIZE + 24)] // n1
	cmp		w13, #6
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X6_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_4x6_lib4c)
#endif
	
	b		103f

101:

	ldr		w13, [sp, #(STACKSIZE + 24)] // n1
	cmp		w13, #7
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X7_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_4x7_lib4c)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X8_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_4x8_lib4c)
#endif

103:



	// prefetch
//	ldr		x8, [sp, #(STACKSIZE + 0)] // D
//	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
//	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
//	INNER_PREFETCH_4X8_LIB
#else
//	CALL(inner_prefetch_4x8_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // ldc
	lsl		w11, w11, #3 // 8*sdc
	ldr		w12, [sp, #(STACKSIZE + 16)] // m1
	ldr		w13, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X8_VS_LIB
#else
	CALL(inner_scale_ab_4x8_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd
	ldr		w10, [sp, #(STACKSIZE + 16)] // m1
	ldr		w11, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_VS_LIB
#else
	CALL(inner_store_4x8_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nn_4x8_vs_lib4ccc)






//                                 w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16
// void kernel_dgemm_nn_4x8_libcccc(int kmax, double *alpha, double *A, int lda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dgemm_nn_4x8_libcccc)
	FUN_START(kernel_dgemm_nn_4x8_libcccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // lda
	lsl		w10, w10, #3 // 8*lda
	mov		x11, x4 // B
	mov		w12, w5 // sdb
	lsl		w12, w12, #3 // 8*sdb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X8_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_4x8_libcc)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X8_LIB
#else
	CALL(inner_scale_ab_4x8_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_LIB
#else
	CALL(inner_store_4x8_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nn_4x8_libcccc)






//                                     w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16    sp+24   sp+32
// void kernel_dgemm_nn_4x8_vs_libcccc(int kmax, double *alpha, double *A, int lda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dgemm_nn_4x8_vs_libcccc)
	FUN_START(kernel_dgemm_nn_4x8_vs_libcccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // lda
	lsl		w10, w10, #3 // 8*lda
	mov		x11, x4 // B
	mov		w12, w5 // sdb
	lsl		w12, w12, #3 // 8*sdb
	ldr		w13, [sp, #(STACKSIZE + 24)] // m1

	ldr		w14, [sp, #(STACKSIZE + 32)] // n1

	cmp		w14, #5
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X5_VS_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_4x5_vs_libcc)
#endif

	b		103f

100:

	cmp		w14, #6
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X6_VS_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_4x6_vs_libcc)
#endif
	
	b		103f

101:

	cmp		w14, #7
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X7_VS_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_4x7_vs_libcc)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X8_VS_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_4x8_vs_libcc)
#endif

103:



	// prefetch
//	ldr		x8, [sp, #(STACKSIZE + 8)] // D
//	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
//	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*sdc
	ldr		w12, [sp, #(STACKSIZE + 24)] // m1
	ldr		w13, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X8_VS_LIB
#else
	CALL(inner_scale_ab_4x8_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*sdd
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

	FUN_END(kernel_dgemm_nn_4x8_vs_libcccc)






//                                  w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16
// void kernel_dgemm_tt_4x8_libc4cc(int kmax, double *alpha, double *A, int lda, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dgemm_tt_4x8_libc4cc)
	FUN_START(kernel_dgemm_tt_4x8_libc4cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x4 // B
	mov		w10, w5 // sdb
	lsl		w10, w10, #5 // 32*sdb
	mov		x11, x2 // A
	mov		w12, w3 // lda
	lsl		w12, w12, #3 // 8*lda

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_8x4_lib4c)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*sdc

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
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_LIB
#else
	CALL(inner_store_4x8_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_tt_4x8_libc4cc)





//                                     w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16    sp+24   sp+32
// void kernel_dgemm_tt_4x8_vs_libc4cc(int kmax, double *alpha, double *A, int lda, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dgemm_tt_4x8_vs_libc4cc)
	FUN_START(kernel_dgemm_tt_4x8_vs_libc4cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x4 // B
	mov		w10, w5 // sdb
	lsl		w10, w10, #5 // 32*sdb
	mov		x11, x2 // A
	mov		w12, w3 // lda
	lsl		w12, w12, #3 // 8*lda

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
	// TODO prefetch vs
//	ldr		x8, [sp, #(STACKSIZE + 8)] // D
//	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
//	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*sdc
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
	lsl		w9, w9, #3 // 8*sdd
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

	FUN_END(kernel_dgemm_tt_4x8_vs_libc4cc)





//                                 w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16
// void kernel_dgemm_tt_4x8_libcccc(int kmax, double *alpha, double *A, int lda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dgemm_tt_4x8_libcccc)
	FUN_START(kernel_dgemm_tt_4x8_libcccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x4 // B
	mov		w10, w5 // ldb
	lsl		w10, w10, #3 // 8*ldb
	mov		x11, x2 // A
	mov		w12, w3 // sda
	lsl		w12, w12, #3 // 8*sda

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X4_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_8x4_libcc)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*sdc

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
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_LIB
#else
	CALL(inner_store_4x8_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_tt_4x8_libcccc)





//                                    w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16    sp+24   sp+32
// void kernel_dgemm_tt_4x8_vs_libcccc(int kmax, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dgemm_tt_4x8_vs_libcccc)
	FUN_START(kernel_dgemm_tt_4x8_vs_libcccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x4 // B
	mov		w10, w5 // ldb
	lsl		w10, w10, #3 // 8*ldb
	mov		x11, x2 // A
	mov		w12, w3 // sda
	lsl		w12, w12, #3 // 8*sda
	ldr		w13, [sp, #(STACKSIZE + 32)] // n1

	ldr		w14, [sp, #(STACKSIZE + 24)] // m1

	cmp		w14, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X1_VS_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_8x1_vs_libcc)
#endif

	b		103f

100:

	cmp		w14, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X2_VS_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_8x2_vs_libcc)
#endif
	
	b		103f

101:

	cmp		w14, #3
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X3_VS_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_8x3_vs_libcc)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X4_VS_LIBCC
#else
	CALL(inner_kernel_gemm_add_nn_8x4_vs_libcc)
#endif

103:



	// prefetch
	// TODO prefetch vs
//	ldr		x8, [sp, #(STACKSIZE + 8)] // D
//	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
//	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*sdc
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
	lsl		w9, w9, #3 // 8*sdd
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

	FUN_END(kernel_dgemm_tt_4x8_vs_libcccc)





//                                     w0     w1     x2             x3         w4       x5         x6       x7            sp+0       sp+8     sp+16      sp+24    sp+32        sp+40
// void kernel_dgemm_nt_8xn_p0_lib44cc(int n, int k, double *alpha, double *A, int sda, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd, double *A_p, double *B_p)

	.align	4
	GLOB(kernel_dgemm_nt_8xn_p0_lib44cc)
	FUN_START(kernel_dgemm_nt_8xn_p0_lib44cc)
	


	PROLOGUE



	// loop over n
//1001:
	cmp		w0, #4
	ble		1000f // consider clean-up loop

1001:



	ZERO_ACC



	// prefetch C
//	mov		x8, x7 // beta
	ld1		{v29.2d}, [x7]
	fcmpe	d29, #0.0
	beq		100f

	ldr		x8, [sp, #(STACKSIZE + 0)] // C
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldc
	lsl		w9, w9, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_PREFETCH_8X4_LIB
#else
	CALL(inner_prefetch_8x4_lib)
#endif

100:


	// call inner kernel gemm nt
	mov		w8, w1 // kmax
	mov		x9, x3 // A
	mov		w10, w4 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x5 // B

	// TODO
//	ldr		x12, [sp, #(STACKSIZE + 16)] // A_p
//	ldr		x13, [sp, #(STACKSIZE + 24)] // B_p
	mov		x12, x3 // A
	mov		w13, w6 // sdb
	lsl		w13, w13, #5 // 32*sdb
	add		x13, x13, x5

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_P0_LIB4
//	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_p0_lib4)
//	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 16)] // D
	ldr		w9, [sp, #(STACKSIZE + 24)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_8X4_LIB
#else
	CALL(inner_prefetch_8x4_lib)
#endif



	ld1		{v29.2d}, [x7] // beta
	fmov	d28, 1.0e+0 // 1.0
	fcmpe	d29, d28
	bne		100f

	ldr		x8, [sp, #(STACKSIZE + 0)] // C
	ldr		x9, [sp, #(STACKSIZE + 16)] // D
	cmp		x8, x9
	bne		100f

	ldr		w8, [sp, #(STACKSIZE + 8)] // ldc
	ldr		w9, [sp, #(STACKSIZE + 24)] // ldd
	cmp		w8, w9
	bne		100f

	//
	mov		x8, x2 // alpha
	ldr		x9, [sp, #(STACKSIZE + 16)] // D
	ldr		w10, [sp, #(STACKSIZE + 24)] // ldd
	lsl		w10, w10, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_A1_STORE_8X4_LIB
#else
	CALL(inner_scale_a1_store_8x4_lib)
#endif

	b		101f

100:

	// call inner blend for generic alpha and beta
	mov		x8, x2 // alpha
	mov		x9, x7 // beta
	ldr		x10, [sp, #(STACKSIZE + 0)] // C
	ldr		w11, [sp, #(STACKSIZE + 8)] // ldc
	lsl		w11, w11, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB
#else
	CALL(inner_scale_ab_8x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 16)] // D
	ldr		w9, [sp, #(STACKSIZE + 24)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif

101:



	// loop increments
	// n
	sub		w0, w0, #4
	// B
	mov		w8, w6 // sdb
	lsl		w8, w8, #5 // 32*sdb
	add		x5, x8, x5
	// C
	ldr		w8, [sp, #(STACKSIZE + 8)] // ldc
	lsl		w8, w8, #5 // 32*ldb
	ldr		x9, [sp, #(STACKSIZE + 0)] // C
	add		x9, x8, x9
	str		x9, [sp, #(STACKSIZE + 0)] // C
	// D
	ldr		w8, [sp, #(STACKSIZE + 24)] // ldd
	lsl		w8, w8, #5 // 32*ldb
	ldr		x9, [sp, #(STACKSIZE + 16)] // D
	add		x9, x8, x9
	str		x9, [sp, #(STACKSIZE + 16)] // D



	cmp		w0, #4
	bgt		1001b // consider clean-up loop



1000:
	// TODO clean-up
	cmp		w0, #3
//	cmp		w0, #0
	ble		1000f // consider clean-up loop



	ZERO_ACC



	// prefetch C
//	mov		x8, x7 // beta
	ld1		{v29.2d}, [x7]
	fcmpe	d29, #0.0
	beq		100f

	ldr		x8, [sp, #(STACKSIZE + 0)] // C
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldc
	lsl		w9, w9, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_PREFETCH_8X4_LIB
#else
	CALL(inner_prefetch_8x4_lib)
#endif

100:


	// call inner kernel gemm nt
	mov		w8, w1 // kmax
	mov		x9, x3 // A
	mov		w10, w4 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x5 // B

	// TODO
	ldr		x12, [sp, #(STACKSIZE + 32)] // A_p
	ldr		x13, [sp, #(STACKSIZE + 40)] // B_p
//	mov		x12, x3 // A
//	mov		w13, w6 // sdb
//	lsl		w13, w13, #5 // 32*sdb
//	add		x13, x13, x5

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_PL_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_pl_lib4)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 16)] // D
	ldr		w9, [sp, #(STACKSIZE + 24)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_8X4_LIB
#else
	CALL(inner_prefetch_8x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x2 // alpha
	mov		x9, x7 // beta
	ldr		x10, [sp, #(STACKSIZE + 0)] // C
	ldr		w11, [sp, #(STACKSIZE + 8)] // ldc
	lsl		w11, w11, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB
#else
	CALL(inner_scale_ab_8x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 16)] // D
	ldr		w9, [sp, #(STACKSIZE + 24)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif



#if 0
	// loop increments
	// n
	sub		w0, w0, #4
	// B
	mov		w8, w6 // sdb
	lsl		w8, w8, #5 // 32*sdb
	add		x5, x8, x5
	// C
	ldr		w8, [sp, #(STACKSIZE + 8)] // ldc
	lsl		w8, w8, #5 // 32*ldb
	ldr		x9, [sp, #(STACKSIZE + 0)] // C
	add		x9, x8, x9
	str		x9, [sp, #(STACKSIZE + 0)] // C
	// D
	ldr		w8, [sp, #(STACKSIZE + 24)] // ldd
	lsl		w8, w8, #5 // 32*ldb
	ldr		x9, [sp, #(STACKSIZE + 16)] // D
	add		x9, x8, x9
	str		x9, [sp, #(STACKSIZE + 16)] // D
#endif



	b		2000f // return



1000:
	// TODO clean-up
	cmp		w0, #0
	ble		2000f // consider clean-up loop



	ZERO_ACC



#if 0
	// prefetch C
	// TODO vs
//	mov		x8, x7 // beta
	ld1		{v29.2d}, [x7]
	fcmpe	d29, #0.0
	beq		100f

	ldr		x8, [sp, #(STACKSIZE + 0)] // C
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldc
	lsl		w9, w9, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_PREFETCH_8X4_LIB
#else
	CALL(inner_prefetch_8x4_lib)
#endif

100:
#endif


	// call inner kernel gemm nt
	mov		w8, w1 // kmax
	mov		x9, x3 // A
	mov		w10, w4 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x5 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif



#if 0
	// prefetch
	// TODO vs
	ldr		x8, [sp, #(STACKSIZE + 16)] // D
	ldr		w9, [sp, #(STACKSIZE + 24)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_8X4_LIB
#else
	CALL(inner_prefetch_8x4_lib)
#endif
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x2 // alpha
	mov		x9, x7 // beta
	ldr		x10, [sp, #(STACKSIZE + 0)] // C
	ldr		w11, [sp, #(STACKSIZE + 8)] // ldc
	lsl		w11, w11, #3 // 8*sdc
	mov		w12, #8 // m1
	mov		w13, w0 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_VS_LIB
#else
	CALL(inner_scale_ab_8x4_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 16)] // D
	ldr		w9, [sp, #(STACKSIZE + 24)] // ldd
	lsl		w9, w9, #3 // 8*sdd
	mov		w10, #8 // m1
	mov		w11, w0 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB
#else
	CALL(inner_store_8x4_vs_lib)
#endif



#if 0
	// loop increments
	// n
	sub		w0, w0, #4
	// B
	mov		w8, w6 // sdb
	lsl		w8, w8, #5 // 32*sdb
	add		x5, x8, x5
	// C
	ldr		w8, [sp, #(STACKSIZE + 8)] // ldc
	lsl		w8, w8, #5 // 32*ldb
	ldr		x9, [sp, #(STACKSIZE + 0)] // C
	add		x9, x8, x9
	str		x9, [sp, #(STACKSIZE + 0)] // C
	// D
	ldr		w8, [sp, #(STACKSIZE + 24)] // ldd
	lsl		w8, w8, #5 // 32*ldb
	ldr		x9, [sp, #(STACKSIZE + 16)] // D
	add		x9, x8, x9
	str		x9, [sp, #(STACKSIZE + 16)] // D
#endif



2000:

	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nt_8xn_p0_lib44cc)






