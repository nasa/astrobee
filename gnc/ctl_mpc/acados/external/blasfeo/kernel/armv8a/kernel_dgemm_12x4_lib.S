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
// x12   <- 8*ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_12X4_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_12x4_lib4c)
#endif



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x9, x10
	add		x14, x13, x10

	add		x15, x11, x12
	add		x16, x15, x12
	add		x17, x16, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch

	// main loop
1:
	
	// unroll 0
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8
	ldr		x24, [x15], #8
	ins		v30.d[1], x24
	ldr		d31, [x16], #8
	ldr		x25, [x17], #8
	ins		v31.d[1], x25

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	fmla	v5.2d, v25.2d, v31.d[0]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v13.2d, v27.2d, v31.d[0]
	fmla	v20.2d, v28.2d, v31.d[0]
	fmla	v21.2d, v29.2d, v31.d[0]
	fmla	v6.2d, v24.2d, v31.d[1]
	fmla	v7.2d, v25.2d, v31.d[1]
	fmla	v14.2d, v26.2d, v31.d[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	fmla	v22.2d, v28.2d, v31.d[1]
	fmla	v23.2d, v29.2d, v31.d[1]

	// unroll 1
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8
	ldr		x24, [x15], #8
	ins		v30.d[1], x24
	ldr		d31, [x16], #8
	ldr		x25, [x17], #8
	ins		v31.d[1], x25

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	fmla	v5.2d, v25.2d, v31.d[0]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v13.2d, v27.2d, v31.d[0]
	fmla	v20.2d, v28.2d, v31.d[0]
	fmla	v21.2d, v29.2d, v31.d[0]
	fmla	v6.2d, v24.2d, v31.d[1]
	fmla	v7.2d, v25.2d, v31.d[1]
	fmla	v14.2d, v26.2d, v31.d[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	fmla	v22.2d, v28.2d, v31.d[1]
	fmla	v23.2d, v29.2d, v31.d[1]

	// unroll 2
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8
	ldr		x24, [x15], #8
	ins		v30.d[1], x24
	ldr		d31, [x16], #8
	ldr		x25, [x17], #8
	ins		v31.d[1], x25

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	fmla	v5.2d, v25.2d, v31.d[0]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v13.2d, v27.2d, v31.d[0]
	fmla	v20.2d, v28.2d, v31.d[0]
	fmla	v21.2d, v29.2d, v31.d[0]
	fmla	v6.2d, v24.2d, v31.d[1]
	fmla	v7.2d, v25.2d, v31.d[1]
	fmla	v14.2d, v26.2d, v31.d[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	fmla	v22.2d, v28.2d, v31.d[1]
	fmla	v23.2d, v29.2d, v31.d[1]

	// unroll 3
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8
	ldr		x24, [x15], #8
	ins		v30.d[1], x24
	ldr		d31, [x16], #8
	ldr		x25, [x17], #8
	ins		v31.d[1], x25

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	fmla	v5.2d, v25.2d, v31.d[0]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v13.2d, v27.2d, v31.d[0]
	fmla	v20.2d, v28.2d, v31.d[0]
	fmla	v21.2d, v29.2d, v31.d[0]
	fmla	v6.2d, v24.2d, v31.d[1]
	fmla	v7.2d, v25.2d, v31.d[1]
	fmla	v14.2d, v26.2d, v31.d[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	fmla	v22.2d, v28.2d, v31.d[1]
	fmla	v23.2d, v29.2d, v31.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8
	ldr		x24, [x15], #8
	ins		v30.d[1], x24
	ldr		d31, [x16], #8
	ldr		x25, [x17], #8
	ins		v31.d[1], x25

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	fmla	v5.2d, v25.2d, v31.d[0]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v13.2d, v27.2d, v31.d[0]
	fmla	v20.2d, v28.2d, v31.d[0]
	fmla	v21.2d, v29.2d, v31.d[0]
	fmla	v6.2d, v24.2d, v31.d[1]
	fmla	v7.2d, v25.2d, v31.d[1]
	fmla	v14.2d, v26.2d, v31.d[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	fmla	v22.2d, v28.2d, v31.d[1]
	fmla	v23.2d, v29.2d, v31.d[1]

	// unroll 1
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8
	ldr		x24, [x15], #8
	ins		v30.d[1], x24
	ldr		d31, [x16], #8
	ldr		x25, [x17], #8
	ins		v31.d[1], x25

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	fmla	v5.2d, v25.2d, v31.d[0]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v13.2d, v27.2d, v31.d[0]
	fmla	v20.2d, v28.2d, v31.d[0]
	fmla	v21.2d, v29.2d, v31.d[0]
	fmla	v6.2d, v24.2d, v31.d[1]
	fmla	v7.2d, v25.2d, v31.d[1]
	fmla	v14.2d, v26.2d, v31.d[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	fmla	v22.2d, v28.2d, v31.d[1]
	fmla	v23.2d, v29.2d, v31.d[1]

	// unroll 2
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8
	ldr		x24, [x15], #8
	ins		v30.d[1], x24
	ldr		d31, [x16], #8
	ldr		x25, [x17], #8
	ins		v31.d[1], x25

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	fmla	v5.2d, v25.2d, v31.d[0]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v13.2d, v27.2d, v31.d[0]
	fmla	v20.2d, v28.2d, v31.d[0]
	fmla	v21.2d, v29.2d, v31.d[0]
	fmla	v6.2d, v24.2d, v31.d[1]
	fmla	v7.2d, v25.2d, v31.d[1]
	fmla	v14.2d, v26.2d, v31.d[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	fmla	v22.2d, v28.2d, v31.d[1]
	fmla	v23.2d, v29.2d, v31.d[1]

	// unroll 3
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8
	ldr		x24, [x15], #8
	ins		v30.d[1], x24
	ldr		d31, [x16], #8
	ldr		x25, [x17], #8
	ins		v31.d[1], x25

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	fmla	v5.2d, v25.2d, v31.d[0]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v13.2d, v27.2d, v31.d[0]
	fmla	v20.2d, v28.2d, v31.d[0]
	fmla	v21.2d, v29.2d, v31.d[0]
	fmla	v6.2d, v24.2d, v31.d[1]
	fmla	v7.2d, v25.2d, v31.d[1]
	fmla	v14.2d, v26.2d, v31.d[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	fmla	v22.2d, v28.2d, v31.d[1]
	fmla	v23.2d, v29.2d, v31.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8
	ldr		x24, [x15], #8
	ins		v30.d[1], x24
	ldr		d31, [x16], #8
	ldr		x25, [x17], #8
	ins		v31.d[1], x25

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	fmla	v5.2d, v25.2d, v31.d[0]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v13.2d, v27.2d, v31.d[0]
	fmla	v20.2d, v28.2d, v31.d[0]
	fmla	v21.2d, v29.2d, v31.d[0]
	fmla	v6.2d, v24.2d, v31.d[1]
	fmla	v7.2d, v25.2d, v31.d[1]
	fmla	v14.2d, v26.2d, v31.d[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	fmla	v22.2d, v28.2d, v31.d[1]
	fmla	v23.2d, v29.2d, v31.d[1]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_12x4_lib4c)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_12X3_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_12x3_lib4c)
#endif



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x9, x10
	add		x14, x13, x10

	add		x15, x11, x12
	add		x16, x15, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch

	// main loop
1:
	
	// unroll 0
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8
	ldr		x24, [x15], #8
	ins		v30.d[1], x24
	ldr		d31, [x16], #8

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	fmla	v5.2d, v25.2d, v31.d[0]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v13.2d, v27.2d, v31.d[0]
	fmla	v20.2d, v28.2d, v31.d[0]
	fmla	v21.2d, v29.2d, v31.d[0]

	// unroll 1
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8
	ldr		x24, [x15], #8
	ins		v30.d[1], x24
	ldr		d31, [x16], #8

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	fmla	v5.2d, v25.2d, v31.d[0]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v13.2d, v27.2d, v31.d[0]
	fmla	v20.2d, v28.2d, v31.d[0]
	fmla	v21.2d, v29.2d, v31.d[0]

	// unroll 2
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8
	ldr		x24, [x15], #8
	ins		v30.d[1], x24
	ldr		d31, [x16], #8

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	fmla	v5.2d, v25.2d, v31.d[0]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v13.2d, v27.2d, v31.d[0]
	fmla	v20.2d, v28.2d, v31.d[0]
	fmla	v21.2d, v29.2d, v31.d[0]

	// unroll 3
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8
	ldr		x24, [x15], #8
	ins		v30.d[1], x24
	ldr		d31, [x16], #8

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	fmla	v5.2d, v25.2d, v31.d[0]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v13.2d, v27.2d, v31.d[0]
	fmla	v20.2d, v28.2d, v31.d[0]
	fmla	v21.2d, v29.2d, v31.d[0]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8
	ldr		x24, [x15], #8
	ins		v30.d[1], x24
	ldr		d31, [x16], #8

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	fmla	v5.2d, v25.2d, v31.d[0]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v13.2d, v27.2d, v31.d[0]
	fmla	v20.2d, v28.2d, v31.d[0]
	fmla	v21.2d, v29.2d, v31.d[0]

	// unroll 1
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8
	ldr		x24, [x15], #8
	ins		v30.d[1], x24
	ldr		d31, [x16], #8

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	fmla	v5.2d, v25.2d, v31.d[0]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v13.2d, v27.2d, v31.d[0]
	fmla	v20.2d, v28.2d, v31.d[0]
	fmla	v21.2d, v29.2d, v31.d[0]

	// unroll 2
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8
	ldr		x24, [x15], #8
	ins		v30.d[1], x24
	ldr		d31, [x16], #8

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	fmla	v5.2d, v25.2d, v31.d[0]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v13.2d, v27.2d, v31.d[0]
	fmla	v20.2d, v28.2d, v31.d[0]
	fmla	v21.2d, v29.2d, v31.d[0]

	// unroll 3
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8
	ldr		x24, [x15], #8
	ins		v30.d[1], x24
	ldr		d31, [x16], #8

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	fmla	v5.2d, v25.2d, v31.d[0]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v13.2d, v27.2d, v31.d[0]
	fmla	v20.2d, v28.2d, v31.d[0]
	fmla	v21.2d, v29.2d, v31.d[0]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8
	ldr		x24, [x15], #8
	ins		v30.d[1], x24
	ldr		d31, [x16], #8

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	fmla	v5.2d, v25.2d, v31.d[0]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v13.2d, v27.2d, v31.d[0]
	fmla	v20.2d, v28.2d, v31.d[0]
	fmla	v21.2d, v29.2d, v31.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_12x3_lib4c)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_12X2_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_12x2_lib4c)
#endif



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x9, x10
	add		x14, x13, x10

	add		x15, x11, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch

	// main loop
1:
	
	// unroll 0
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8
	ldr		x24, [x15], #8
	ins		v30.d[1], x24

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]

	// unroll 1
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8
	ldr		x24, [x15], #8
	ins		v30.d[1], x24

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]

	// unroll 2
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8
	ldr		x24, [x15], #8
	ins		v30.d[1], x24

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]

	// unroll 3
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8
	ldr		x24, [x15], #8
	ins		v30.d[1], x24

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8
	ldr		x24, [x15], #8
	ins		v30.d[1], x24

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]

	// unroll 1
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8
	ldr		x24, [x15], #8
	ins		v30.d[1], x24

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]

	// unroll 2
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8
	ldr		x24, [x15], #8
	ins		v30.d[1], x24

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]

	// unroll 3
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8
	ldr		x24, [x15], #8
	ins		v30.d[1], x24

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8
	ldr		x24, [x15], #8
	ins		v30.d[1], x24

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_12x2_lib4c)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_12X1_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_12x1_lib4c)
#endif



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x9, x10
	add		x14, x13, x10

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop


	// prefetch

	// main loop
1:
	
	// unroll 0
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]

	// unroll 1
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]

	// unroll 2
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]

	// unroll 3
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]

	sub		w8, w8, #4
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]

	// unroll 1
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]

	// unroll 2
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]

	// unroll 3
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d30, [x11], #8

	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_12x1_lib4c)
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
	.macro INNER_EDGE_TRSM_LLN_ONE_12X4_LIB
#else
	.align 4
	FUN_START(inner_edge_trsm_lln_one_12x4_lib)
#endif

	ldp		q24, q25, [x8, #0] // E0[0+4*0]
	ldp		q26, q27, [x8, #32] // E1[0+4*0]
	ldp		q28, q29, [x8, #64] // E2[0+4*0]
	add		x8, x8, x9
	ins		v24.d[0], xzr
	fmls	v0.2d, v24.2d, v0.d[0]
	fmls	v1.2d, v25.2d, v0.d[0]
	fmls	v8.2d, v26.2d, v0.d[0]
	fmls	v9.2d, v27.2d, v0.d[0]
	fmls	v16.2d, v28.2d, v0.d[0]
	fmls	v17.2d, v29.2d, v0.d[0]
	fmls	v2.2d, v24.2d, v2.d[0]
	fmls	v3.2d, v25.2d, v2.d[0]
	fmls	v10.2d, v26.2d, v2.d[0]
	fmls	v11.2d, v27.2d, v2.d[0]
	fmls	v18.2d, v28.2d, v2.d[0]
	fmls	v19.2d, v29.2d, v2.d[0]
	fmls	v4.2d, v24.2d, v4.d[0]
	fmls	v5.2d, v25.2d, v4.d[0]
	fmls	v12.2d, v26.2d, v4.d[0]
	fmls	v13.2d, v27.2d, v4.d[0]
	fmls	v20.2d, v28.2d, v4.d[0]
	fmls	v21.2d, v29.2d, v4.d[0]
	fmls	v6.2d, v24.2d, v6.d[0]
	fmls	v7.2d, v25.2d, v6.d[0]
	fmls	v14.2d, v26.2d, v6.d[0]
	fmls	v15.2d, v27.2d, v6.d[0]
	fmls	v22.2d, v28.2d, v6.d[0]
	fmls	v23.2d, v29.2d, v6.d[0]

	ldr		q25, [x8, #16] // E[2+4*1]
	ldp		q26, q27, [x8, #32] // E1[0+4*1]
	ldp		q28, q29, [x8, #64] // E2[0+4*1]
	add		x8, x8, x9
	fmls	v1.2d, v25.2d, v0.d[1]
	fmls	v8.2d, v26.2d, v0.d[1]
	fmls	v9.2d, v27.2d, v0.d[1]
	fmls	v16.2d, v28.2d, v0.d[1]
	fmls	v17.2d, v29.2d, v0.d[1]
	fmls	v3.2d, v25.2d, v2.d[1]
	fmls	v10.2d, v26.2d, v2.d[1]
	fmls	v11.2d, v27.2d, v2.d[1]
	fmls	v18.2d, v28.2d, v2.d[1]
	fmls	v19.2d, v29.2d, v2.d[1]
	fmls	v5.2d, v25.2d, v4.d[1]
	fmls	v12.2d, v26.2d, v4.d[1]
	fmls	v13.2d, v27.2d, v4.d[1]
	fmls	v20.2d, v28.2d, v4.d[1]
	fmls	v21.2d, v29.2d, v4.d[1]
	fmls	v7.2d, v25.2d, v6.d[1]
	fmls	v14.2d, v26.2d, v6.d[1]
	fmls	v15.2d, v27.2d, v6.d[1]
	fmls	v22.2d, v28.2d, v6.d[1]
	fmls	v23.2d, v29.2d, v6.d[1]

	ldr		q25, [x8, #16] // E[2+4*2]
	ldp		q26, q27, [x8, #32] // E1[0+4*2]
	ldp		q28, q29, [x8, #64] // E2[0+4*2]
	add		x8, x8, x9
	ins		v25.d[0], xzr
	fmls	v1.2d, v25.2d, v1.d[0]
	fmls	v8.2d, v26.2d, v1.d[0]
	fmls	v9.2d, v27.2d, v1.d[0]
	fmls	v16.2d, v28.2d, v1.d[0]
	fmls	v17.2d, v29.2d, v1.d[0]
	fmls	v3.2d, v25.2d, v3.d[0]
	fmls	v10.2d, v26.2d, v3.d[0]
	fmls	v11.2d, v27.2d, v3.d[0]
	fmls	v18.2d, v28.2d, v3.d[0]
	fmls	v19.2d, v29.2d, v3.d[0]
	fmls	v5.2d, v25.2d, v5.d[0]
	fmls	v12.2d, v26.2d, v5.d[0]
	fmls	v13.2d, v27.2d, v5.d[0]
	fmls	v20.2d, v28.2d, v5.d[0]
	fmls	v21.2d, v29.2d, v5.d[0]
	fmls	v7.2d, v25.2d, v7.d[0]
	fmls	v14.2d, v26.2d, v7.d[0]
	fmls	v15.2d, v27.2d, v7.d[0]
	fmls	v22.2d, v28.2d, v7.d[0]
	fmls	v23.2d, v29.2d, v7.d[0]

	ldp		q26, q27, [x8, #32] // E1[0+4*3]
	ldp		q28, q29, [x8, #64] // E2[0+4*3]
	add		x8, x8, x9
	fmls	v8.2d, v26.2d, v1.d[1]
	fmls	v9.2d, v27.2d, v1.d[1]
	fmls	v16.2d, v28.2d, v1.d[1]
	fmls	v17.2d, v29.2d, v1.d[1]
	fmls	v10.2d, v26.2d, v3.d[1]
	fmls	v11.2d, v27.2d, v3.d[1]
	fmls	v18.2d, v28.2d, v3.d[1]
	fmls	v19.2d, v29.2d, v3.d[1]
	fmls	v12.2d, v26.2d, v5.d[1]
	fmls	v13.2d, v27.2d, v5.d[1]
	fmls	v20.2d, v28.2d, v5.d[1]
	fmls	v21.2d, v29.2d, v5.d[1]
	fmls	v14.2d, v26.2d, v7.d[1]
	fmls	v15.2d, v27.2d, v7.d[1]
	fmls	v22.2d, v28.2d, v7.d[1]
	fmls	v23.2d, v29.2d, v7.d[1]

	ldp		q24, q25, [x8, #32] // E1[0+4*0]
	ldp		q26, q27, [x8, #64] // E2[0+4*0]
	add		x8, x8, x9
	ins		v24.d[0], xzr
	fmls	v8.2d, v24.2d, v8.d[0]
	fmls	v9.2d, v25.2d, v8.d[0]
	fmls	v16.2d, v26.2d, v8.d[0]
	fmls	v17.2d, v27.2d, v8.d[0]
	fmls	v10.2d, v24.2d, v10.d[0]
	fmls	v11.2d, v25.2d, v10.d[0]
	fmls	v18.2d, v26.2d, v10.d[0]
	fmls	v19.2d, v27.2d, v10.d[0]
	fmls	v12.2d, v24.2d, v12.d[0]
	fmls	v13.2d, v25.2d, v12.d[0]
	fmls	v20.2d, v26.2d, v12.d[0]
	fmls	v21.2d, v27.2d, v12.d[0]
	fmls	v14.2d, v24.2d, v14.d[0]
	fmls	v15.2d, v25.2d, v14.d[0]
	fmls	v22.2d, v26.2d, v14.d[0]
	fmls	v23.2d, v27.2d, v14.d[0]

	ldr		q25, [x8, #48] // E1[2+4*1]
	ldp		q26, q27, [x8, #64] // E2[0+4*1]
	add		x8, x8, x9
	fmls	v9.2d, v25.2d, v8.d[1]
	fmls	v16.2d, v26.2d, v8.d[1]
	fmls	v17.2d, v27.2d, v8.d[1]
	fmls	v11.2d, v25.2d, v10.d[1]
	fmls	v18.2d, v26.2d, v10.d[1]
	fmls	v19.2d, v27.2d, v10.d[1]
	fmls	v13.2d, v25.2d, v12.d[1]
	fmls	v20.2d, v26.2d, v12.d[1]
	fmls	v21.2d, v27.2d, v12.d[1]
	fmls	v15.2d, v25.2d, v14.d[1]
	fmls	v22.2d, v26.2d, v14.d[1]
	fmls	v23.2d, v27.2d, v14.d[1]

	ldr		q25, [x8, #48] // E1[2+4*2]
	ldp		q26, q27, [x8, #64] // E2[0+4*2]
	add		x8, x8, x9
	ins		v25.d[0], xzr
	fmls	v9.2d, v25.2d, v9.d[0]
	fmls	v16.2d, v26.2d, v9.d[0]
	fmls	v17.2d, v27.2d, v9.d[0]
	fmls	v11.2d, v25.2d, v11.d[0]
	fmls	v18.2d, v26.2d, v11.d[0]
	fmls	v19.2d, v27.2d, v11.d[0]
	fmls	v13.2d, v25.2d, v13.d[0]
	fmls	v20.2d, v26.2d, v13.d[0]
	fmls	v21.2d, v27.2d, v13.d[0]
	fmls	v15.2d, v25.2d, v15.d[0]
	fmls	v22.2d, v26.2d, v15.d[0]
	fmls	v23.2d, v27.2d, v15.d[0]

	ldp		q26, q27, [x8, #64] // E2[0+4*3]
	add		x8, x8, x9
	fmls	v16.2d, v26.2d, v9.d[1]
	fmls	v17.2d, v27.2d, v9.d[1]
	fmls	v18.2d, v26.2d, v11.d[1]
	fmls	v19.2d, v27.2d, v11.d[1]
	fmls	v20.2d, v26.2d, v13.d[1]
	fmls	v21.2d, v27.2d, v13.d[1]
	fmls	v22.2d, v26.2d, v15.d[1]
	fmls	v23.2d, v27.2d, v15.d[1]

	ldp		q24, q25, [x8, #64] // E2[0+4*4]
	add		x8, x8, x9
	ins		v24.d[0], xzr
	fmls	v16.2d, v24.2d, v16.d[0]
	fmls	v17.2d, v25.2d, v16.d[0]
	fmls	v18.2d, v24.2d, v18.d[0]
	fmls	v19.2d, v25.2d, v18.d[0]
	fmls	v20.2d, v24.2d, v20.d[0]
	fmls	v21.2d, v25.2d, v20.d[0]
	fmls	v22.2d, v24.2d, v22.d[0]
	fmls	v23.2d, v25.2d, v22.d[0]

	ldr		q25, [x8, #80] // E2[2+4*5]
	add		x8, x8, x9
	fmls	v17.2d, v25.2d, v16.d[1]
	fmls	v19.2d, v25.2d, v18.d[1]
	fmls	v21.2d, v25.2d, v20.d[1]
	fmls	v23.2d, v25.2d, v22.d[1]

	ldr		q25, [x8, #80] // E2[2+4*6]
//	add		x8, x8, x9
	ins		v25.d[0], xzr
	fmls	v17.2d, v25.2d, v17.d[0]
	fmls	v19.2d, v25.2d, v19.d[0]
	fmls	v21.2d, v25.2d, v21.d[0]
	fmls	v23.2d, v25.2d, v23.d[0]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsm_lln_one_12x4_lib)
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
	.macro INNER_EDGE_TRSM_RLT_INV_12X4_LIB
#else
	.align 4
	FUN_START(inner_edge_trsm_rlt_inv_12x4_lib)
#endif
	
	ldr			d24, [x10, #0] // E_inv[0]
	fmul		v0.2d, v0.2d, v24.d[0]
	fmul		v1.2d, v1.2d, v24.d[0]
	fmul		v8.2d, v8.2d, v24.d[0]
	fmul		v9.2d, v9.2d, v24.d[0]
	fmul		v16.2d, v16.2d, v24.d[0]
	fmul		v17.2d, v17.2d, v24.d[0]
	ldr			d24, [x8, #8] // E[1+4*0]
	fmls		v2.2d, v0.2d, v24.d[0]
	fmls		v3.2d, v1.2d, v24.d[0]
	fmls		v10.2d, v8.2d, v24.d[0]
	fmls		v11.2d, v9.2d, v24.d[0]
	fmls		v18.2d, v16.2d, v24.d[0]
	fmls		v19.2d, v17.2d, v24.d[0]
	ldr			d24, [x8, #16] // E[2+4*0]
	fmls		v4.2d, v0.2d, v24.d[0]
	fmls		v5.2d, v1.2d, v24.d[0]
	fmls		v12.2d, v8.2d, v24.d[0]
	fmls		v13.2d, v9.2d, v24.d[0]
	fmls		v20.2d, v16.2d, v24.d[0]
	fmls		v21.2d, v17.2d, v24.d[0]
	ldr			d24, [x8, #24] // E[3+4*0]
	fmls		v6.2d, v0.2d, v24.d[0]
	fmls		v7.2d, v1.2d, v24.d[0]
	fmls		v14.2d, v8.2d, v24.d[0]
	fmls		v15.2d, v9.2d, v24.d[0]
	fmls		v22.2d, v16.2d, v24.d[0]
	fmls		v23.2d, v17.2d, v24.d[0]
	add			x8, x8, x9

	ldr			d24, [x10, #8] // E_inv[1]
	fmul		v2.2d, v2.2d, v24.d[0]
	fmul		v3.2d, v3.2d, v24.d[0]
	fmul		v10.2d, v10.2d, v24.d[0]
	fmul		v11.2d, v11.2d, v24.d[0]
	fmul		v18.2d, v18.2d, v24.d[0]
	fmul		v19.2d, v19.2d, v24.d[0]
	ldr			d24, [x8, #16] // E[2+4*1]
	fmls		v4.2d, v2.2d, v24.d[0]
	fmls		v5.2d, v3.2d, v24.d[0]
	fmls		v12.2d, v10.2d, v24.d[0]
	fmls		v13.2d, v11.2d, v24.d[0]
	fmls		v20.2d, v18.2d, v24.d[0]
	fmls		v21.2d, v19.2d, v24.d[0]
	ldr			d24, [x8, #24] // E[3+4*1]
	fmls		v6.2d, v2.2d, v24.d[0]
	fmls		v7.2d, v3.2d, v24.d[0]
	fmls		v14.2d, v10.2d, v24.d[0]
	fmls		v15.2d, v11.2d, v24.d[0]
	fmls		v22.2d, v18.2d, v24.d[0]
	fmls		v23.2d, v19.2d, v24.d[0]
	add			x8, x8, x9

	ldr			d24, [x10, #16] // E_inv[2]
	fmul		v4.2d, v4.2d, v24.d[0]
	fmul		v5.2d, v5.2d, v24.d[0]
	fmul		v12.2d, v12.2d, v24.d[0]
	fmul		v13.2d, v13.2d, v24.d[0]
	fmul		v20.2d, v20.2d, v24.d[0]
	fmul		v21.2d, v21.2d, v24.d[0]
	ldr			d24, [x8, #24] // E[3+4*1]
	fmls		v6.2d, v4.2d, v24.d[0]
	fmls		v7.2d, v5.2d, v24.d[0]
	fmls		v14.2d, v12.2d, v24.d[0]
	fmls		v15.2d, v13.2d, v24.d[0]
	fmls		v22.2d, v20.2d, v24.d[0]
	fmls		v23.2d, v21.2d, v24.d[0]
//	add			x8, x8, x9

	ldr			d24, [x10, #24] // E_inv[2]
	fmul		v6.2d, v6.2d, v24.d[0]
	fmul		v7.2d, v7.2d, v24.d[0]
	fmul		v14.2d, v14.2d, v24.d[0]
	fmul		v15.2d, v15.2d, v24.d[0]
	fmul		v22.2d, v22.2d, v24.d[0]
	fmul		v23.2d, v23.2d, v24.d[0]
//	add			x8, x8, x9

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsm_rlt_inv_12x4_lib)
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
	.macro INNER_EDGE_TRSM_RLT_INV_12X4_VS_LIB
#else
	.align 4
	FUN_START(inner_edge_trsm_rlt_inv_12x4_vs_lib)
#endif
	
	// first column
	ldr			d24, [x10, #0] // E_inv[0]
	fmul		v0.2d, v0.2d, v24.d[0]
	fmul		v1.2d, v1.2d, v24.d[0]
	fmul		v8.2d, v8.2d, v24.d[0]
	fmul		v9.2d, v9.2d, v24.d[0]
	fmul		v16.2d, v16.2d, v24.d[0]
	fmul		v17.2d, v17.2d, v24.d[0]
	cmp			w11, #2
	blt			0f // return

	// second column
	ldr			d24, [x8, #8] // E[1+4*0]
	fmls		v2.2d, v0.2d, v24.d[0]
	fmls		v3.2d, v1.2d, v24.d[0]
	fmls		v10.2d, v8.2d, v24.d[0]
	fmls		v11.2d, v9.2d, v24.d[0]
	fmls		v18.2d, v16.2d, v24.d[0]
	fmls		v19.2d, v17.2d, v24.d[0]
	ldr			d24, [x10, #8] // E_inv[1]
	fmul		v2.2d, v2.2d, v24.d[0]
	fmul		v3.2d, v3.2d, v24.d[0]
	fmul		v10.2d, v10.2d, v24.d[0]
	fmul		v11.2d, v11.2d, v24.d[0]
	fmul		v18.2d, v18.2d, v24.d[0]
	fmul		v19.2d, v19.2d, v24.d[0]
	cmp			w11, #3
	blt			0f // return

	// third column
	add			x12, x8, x9
	ldr			d24, [x8, #16] // E[2+4*0]
	fmls		v4.2d, v0.2d, v24.d[0]
	fmls		v5.2d, v1.2d, v24.d[0]
	fmls		v12.2d, v8.2d, v24.d[0]
	fmls		v13.2d, v9.2d, v24.d[0]
	fmls		v20.2d, v16.2d, v24.d[0]
	fmls		v21.2d, v17.2d, v24.d[0]
	ldr			d24, [x12, #16] // E[2+4*1]
	fmls		v4.2d, v2.2d, v24.d[0]
	fmls		v5.2d, v3.2d, v24.d[0]
	fmls		v12.2d, v10.2d, v24.d[0]
	fmls		v13.2d, v11.2d, v24.d[0]
	fmls		v20.2d, v18.2d, v24.d[0]
	fmls		v21.2d, v19.2d, v24.d[0]
	ldr			d24, [x10, #16] // E_inv[2]
	fmul		v4.2d, v4.2d, v24.d[0]
	fmul		v5.2d, v5.2d, v24.d[0]
	fmul		v12.2d, v12.2d, v24.d[0]
	fmul		v13.2d, v13.2d, v24.d[0]
	fmul		v20.2d, v20.2d, v24.d[0]
	fmul		v21.2d, v21.2d, v24.d[0]
	cmp			w11, #4
	blt			0f // return

	// forth column
	add			x13, x12, x9
	ldr			d24, [x8, #24] // E[3+4*0]
	fmls		v6.2d, v0.2d, v24.d[0]
	fmls		v7.2d, v1.2d, v24.d[0]
	fmls		v14.2d, v8.2d, v24.d[0]
	fmls		v15.2d, v9.2d, v24.d[0]
	fmls		v22.2d, v16.2d, v24.d[0]
	fmls		v23.2d, v17.2d, v24.d[0]
	ldr			d24, [x12, #24] // E[3+4*1]
	fmls		v6.2d, v2.2d, v24.d[0]
	fmls		v7.2d, v3.2d, v24.d[0]
	fmls		v14.2d, v10.2d, v24.d[0]
	fmls		v15.2d, v11.2d, v24.d[0]
	fmls		v22.2d, v18.2d, v24.d[0]
	fmls		v23.2d, v19.2d, v24.d[0]
	ldr			d24, [x13, #24] // E[3+4*2]
	fmls		v6.2d, v4.2d, v24.d[0]
	fmls		v7.2d, v5.2d, v24.d[0]
	fmls		v14.2d, v12.2d, v24.d[0]
	fmls		v15.2d, v13.2d, v24.d[0]
	fmls		v22.2d, v20.2d, v24.d[0]
	fmls		v23.2d, v21.2d, v24.d[0]
	ldr			d24, [x10, #24] // E_inv[3]
	fmul		v6.2d, v6.2d, v24.d[0]
	fmul		v7.2d, v7.2d, v24.d[0]
	fmul		v14.2d, v14.2d, v24.d[0]
	fmul		v15.2d, v15.2d, v24.d[0]
	fmul		v22.2d, v22.2d, v24.d[0]
	fmul		v23.2d, v23.2d, v24.d[0]

0:
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsm_rlt_inv_12x4_vs_lib)
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
	.macro INNER_SCALE_AB_12X4_LIB
#else
	.align	4
	FUN_START(inner_scale_ab_12x4_lib)
#endif

	ld1		{v28.2d}, [x8]

	ld1		{v30.2d}, [x9]

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
	fmul	v16.2d, v16.2d, v28.d[0]
	fmul	v17.2d, v17.2d, v28.d[0]
	fmul	v18.2d, v18.2d, v28.d[0]
	fmul	v19.2d, v19.2d, v28.d[0]
	fmul	v20.2d, v20.2d, v28.d[0]
	fmul	v21.2d, v21.2d, v28.d[0]
	fmul	v22.2d, v22.2d, v28.d[0]
	fmul	v23.2d, v23.2d, v28.d[0]

	fcmpe	d30, #0.0
	beq		0f

	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	ldp		q28, q29, [x10, #64]
	add		x10, x10, x11
	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]

	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	ldp		q28, q29, [x10, #64]
	add		x10, x10, x11
	fmla	v2.2d, v24.2d, v30.d[0]
	fmla	v3.2d, v25.2d, v30.d[0]
	fmla	v10.2d, v26.2d, v30.d[0]
	fmla	v11.2d, v27.2d, v30.d[0]
	fmla	v18.2d, v28.2d, v30.d[0]
	fmla	v19.2d, v29.2d, v30.d[0]

	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	ldp		q28, q29, [x10, #64]
	add		x10, x10, x11
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	fmla	v12.2d, v26.2d, v30.d[0]
	fmla	v13.2d, v27.2d, v30.d[0]
	fmla	v20.2d, v28.2d, v30.d[0]
	fmla	v21.2d, v29.2d, v30.d[0]

	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	ldp		q28, q29, [x10, #64]
	add		x10, x10, x11
	fmla	v6.2d, v24.2d, v30.d[0]
	fmla	v7.2d, v25.2d, v30.d[0]
	fmla	v14.2d, v26.2d, v30.d[0]
	fmla	v15.2d, v27.2d, v30.d[0]
	fmla	v22.2d, v28.2d, v30.d[0]
	fmla	v23.2d, v29.2d, v30.d[0]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_12x4_lib)
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
	.macro INNER_SCALE_AB_12X4_VS_LIB
#else
	.align	4
	FUN_START(inner_scale_ab_12x4_vs_lib)
#endif

	ld1		{v28.2d}, [x8]

	ld1		{v30.2d}, [x9]

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
	fmul	v16.2d, v16.2d, v28.d[0]
	fmul	v17.2d, v17.2d, v28.d[0]
	fmul	v18.2d, v18.2d, v28.d[0]
	fmul	v19.2d, v19.2d, v28.d[0]
	fmul	v20.2d, v20.2d, v28.d[0]
	fmul	v21.2d, v21.2d, v28.d[0]
	fmul	v22.2d, v22.2d, v28.d[0]
	fmul	v23.2d, v23.2d, v28.d[0]

	fcmpe	d30, #0.0
	beq		0f

	cmp		w12, #4
	blt		1f

	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	ldp		q28, q29, [x10, #64]
	add		x10, x10, x11
	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]

	cmp		w13, #1
	ble		0f

	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	ldp		q28, q29, [x10, #64]
	add		x10, x10, x11
	fmla	v2.2d, v24.2d, v30.d[0]
	fmla	v3.2d, v25.2d, v30.d[0]
	fmla	v10.2d, v26.2d, v30.d[0]
	fmla	v11.2d, v27.2d, v30.d[0]
	fmla	v18.2d, v28.2d, v30.d[0]
	fmla	v19.2d, v29.2d, v30.d[0]

	cmp		w13, #2
	ble		0f

	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	ldp		q28, q29, [x10, #64]
	add		x10, x10, x11
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	fmla	v12.2d, v26.2d, v30.d[0]
	fmla	v13.2d, v27.2d, v30.d[0]
	fmla	v20.2d, v28.2d, v30.d[0]
	fmla	v21.2d, v29.2d, v30.d[0]

	cmp		w13, #3
	ble		0f

	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	ldp		q28, q29, [x10, #64]
	add		x10, x10, x11
	fmla	v6.2d, v24.2d, v30.d[0]
	fmla	v7.2d, v25.2d, v30.d[0]
	fmla	v14.2d, v26.2d, v30.d[0]
	fmla	v15.2d, v27.2d, v30.d[0]
	fmla	v22.2d, v28.2d, v30.d[0]
	fmla	v23.2d, v29.2d, v30.d[0]

	b 0f

1:
	cmp		w12, #3
	blt		2f

	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	ldr		q28, [x10, #64]
	ldr		d29, [x10, #80]
	add		x10, x10, x11
	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]

	cmp		w13, #1
	ble		0f

	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	ldr		q28, [x10, #64]
	ldr		d29, [x10, #80]
	add		x10, x10, x11
	fmla	v2.2d, v24.2d, v30.d[0]
	fmla	v3.2d, v25.2d, v30.d[0]
	fmla	v10.2d, v26.2d, v30.d[0]
	fmla	v11.2d, v27.2d, v30.d[0]
	fmla	v18.2d, v28.2d, v30.d[0]
	fmla	v19.2d, v29.2d, v30.d[0]

	cmp		w13, #2
	ble		0f

	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	ldr		q28, [x10, #64]
	ldr		d29, [x10, #80]
	add		x10, x10, x11
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	fmla	v12.2d, v26.2d, v30.d[0]
	fmla	v13.2d, v27.2d, v30.d[0]
	fmla	v20.2d, v28.2d, v30.d[0]
	fmla	v21.2d, v29.2d, v30.d[0]

	cmp		w13, #3
	ble		0f

	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	ldr		q28, [x10, #64]
	ldr		d29, [x10, #80]
	add		x10, x10, x11
	fmla	v6.2d, v24.2d, v30.d[0]
	fmla	v7.2d, v25.2d, v30.d[0]
	fmla	v14.2d, v26.2d, v30.d[0]
	fmla	v15.2d, v27.2d, v30.d[0]
	fmla	v22.2d, v28.2d, v30.d[0]
	fmla	v23.2d, v29.2d, v30.d[0]

	b 0f

2:
	cmp		w12, #2
	blt		3f

	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	ldr		q28, [x10, #64]
	add		x10, x10, x11
	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]

	cmp		w13, #1
	ble		0f

	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	ldr		q28, [x10, #64]
	add		x10, x10, x11
	fmla	v2.2d, v24.2d, v30.d[0]
	fmla	v3.2d, v25.2d, v30.d[0]
	fmla	v10.2d, v26.2d, v30.d[0]
	fmla	v11.2d, v27.2d, v30.d[0]
	fmla	v18.2d, v28.2d, v30.d[0]

	cmp		w13, #2
	ble		0f

	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	ldr		q28, [x10, #64]
	add		x10, x10, x11
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	fmla	v12.2d, v26.2d, v30.d[0]
	fmla	v13.2d, v27.2d, v30.d[0]
	fmla	v20.2d, v28.2d, v30.d[0]

	cmp		w13, #3
	ble		0f

	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	ldr		q28, [x10, #64]
	add		x10, x10, x11
	fmla	v6.2d, v24.2d, v30.d[0]
	fmla	v7.2d, v25.2d, v30.d[0]
	fmla	v14.2d, v26.2d, v30.d[0]
	fmla	v15.2d, v27.2d, v30.d[0]
	fmla	v22.2d, v28.2d, v30.d[0]

	b 0f

3:
	cmp		w12, #1
	blt		0f

	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	ldr		d28, [x10, #64]
	add		x10, x10, x11
	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]

	cmp		w13, #1
	ble		0f

	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	ldr		d28, [x10, #64]
	add		x10, x10, x11
	fmla	v2.2d, v24.2d, v30.d[0]
	fmla	v3.2d, v25.2d, v30.d[0]
	fmla	v10.2d, v26.2d, v30.d[0]
	fmla	v11.2d, v27.2d, v30.d[0]
	fmla	v18.2d, v28.2d, v30.d[0]

	cmp		w13, #2
	ble		0f

	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	ldr		d28, [x10, #64]
	add		x10, x10, x11
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	fmla	v12.2d, v26.2d, v30.d[0]
	fmla	v13.2d, v27.2d, v30.d[0]
	fmla	v20.2d, v28.2d, v30.d[0]

	cmp		w13, #3
	ble		0f

	ldp		q24, q25, [x10, #0]
	ldp		q26, q27, [x10, #32]
	ldr		d28, [x10, #64]
	add		x10, x10, x11
	fmla	v6.2d, v24.2d, v30.d[0]
	fmla	v7.2d, v25.2d, v30.d[0]
	fmla	v14.2d, v26.2d, v30.d[0]
	fmla	v15.2d, v27.2d, v30.d[0]
	fmla	v22.2d, v28.2d, v30.d[0]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_12x4_vs_lib)
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
	.macro INNER_SCALE_M1B_12X4_LIB
#else
	.align	4
	FUN_START(inner_scale_m1b_12x4_lib)
#endif

	ld1		{v30.2d}, [x8]

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

	fneg	v16.2d, v16.2d
	fneg	v17.2d, v17.2d
	fneg	v18.2d, v18.2d
	fneg	v19.2d, v19.2d

	fneg	v20.2d, v20.2d
	fneg	v21.2d, v21.2d
	fneg	v22.2d, v22.2d
	fneg	v23.2d, v23.2d

	fcmpe	d30, #0.0
	beq		0f

	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	ldp		q28, q29, [x9, #64]
	add		x9, x9, x10
	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]

	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	ldp		q28, q29, [x9, #64]
	add		x9, x9, x10
	fmla	v2.2d, v24.2d, v30.d[0]
	fmla	v3.2d, v25.2d, v30.d[0]
	fmla	v10.2d, v26.2d, v30.d[0]
	fmla	v11.2d, v27.2d, v30.d[0]
	fmla	v18.2d, v28.2d, v30.d[0]
	fmla	v19.2d, v29.2d, v30.d[0]

	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	ldp		q28, q29, [x9, #64]
	add		x9, x9, x10
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	fmla	v12.2d, v26.2d, v30.d[0]
	fmla	v13.2d, v27.2d, v30.d[0]
	fmla	v20.2d, v28.2d, v30.d[0]
	fmla	v21.2d, v29.2d, v30.d[0]

	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	ldp		q28, q29, [x9, #64]
	add		x9, x9, x10
	fmla	v6.2d, v24.2d, v30.d[0]
	fmla	v7.2d, v25.2d, v30.d[0]
	fmla	v14.2d, v26.2d, v30.d[0]
	fmla	v15.2d, v27.2d, v30.d[0]
	fmla	v22.2d, v28.2d, v30.d[0]
	fmla	v23.2d, v29.2d, v30.d[0]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m1b_12x4_lib)
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
	.macro INNER_SCALE_M1B_12X4_VS_LIB
#else
	.align	4
	FUN_START(inner_scale_m1b_12x4_vs_lib)
#endif

	ld1		{v30.2d}, [x8]

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

	fneg	v16.2d, v16.2d
	fneg	v17.2d, v17.2d
	fneg	v18.2d, v18.2d
	fneg	v19.2d, v19.2d

	fneg	v20.2d, v20.2d
	fneg	v21.2d, v21.2d
	fneg	v22.2d, v22.2d
	fneg	v23.2d, v23.2d

	fcmpe	d30, #0.0
	beq		0f

	cmp		w11, #4
	blt		1f

	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	ldp		q28, q29, [x9, #64]
	add		x9, x9, x10
	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]

	cmp		w12, #1
	ble		0f

	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	ldp		q28, q29, [x9, #64]
	add		x9, x9, x10
	fmla	v2.2d, v24.2d, v30.d[0]
	fmla	v3.2d, v25.2d, v30.d[0]
	fmla	v10.2d, v26.2d, v30.d[0]
	fmla	v11.2d, v27.2d, v30.d[0]
	fmla	v18.2d, v28.2d, v30.d[0]
	fmla	v19.2d, v29.2d, v30.d[0]

	cmp		w12, #2
	ble		0f

	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	ldp		q28, q29, [x9, #64]
	add		x9, x9, x10
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	fmla	v12.2d, v26.2d, v30.d[0]
	fmla	v13.2d, v27.2d, v30.d[0]
	fmla	v20.2d, v28.2d, v30.d[0]
	fmla	v21.2d, v29.2d, v30.d[0]

	cmp		w12, #3
	ble		0f

	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	ldp		q28, q29, [x9, #64]
	add		x9, x9, x10
	fmla	v6.2d, v24.2d, v30.d[0]
	fmla	v7.2d, v25.2d, v30.d[0]
	fmla	v14.2d, v26.2d, v30.d[0]
	fmla	v15.2d, v27.2d, v30.d[0]
	fmla	v22.2d, v28.2d, v30.d[0]
	fmla	v23.2d, v29.2d, v30.d[0]

	b 0f

1:
	cmp		w11, #3
	blt		2f

	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	ldr		q28, [x9, #64]
	ldr		d29, [x9, #80]
	add		x9, x9, x10
	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]

	cmp		w12, #1
	ble		0f

	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	ldr		q28, [x9, #64]
	ldr		d29, [x9, #80]
	add		x9, x9, x10
	fmla	v2.2d, v24.2d, v30.d[0]
	fmla	v3.2d, v25.2d, v30.d[0]
	fmla	v10.2d, v26.2d, v30.d[0]
	fmla	v11.2d, v27.2d, v30.d[0]
	fmla	v18.2d, v28.2d, v30.d[0]
	fmla	v19.2d, v29.2d, v30.d[0]

	cmp		w12, #2
	ble		0f

	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	ldr		q28, [x9, #64]
	ldr		d29, [x9, #80]
	add		x9, x9, x10
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	fmla	v12.2d, v26.2d, v30.d[0]
	fmla	v13.2d, v27.2d, v30.d[0]
	fmla	v20.2d, v28.2d, v30.d[0]
	fmla	v21.2d, v29.2d, v30.d[0]

	cmp		w12, #3
	ble		0f

	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	ldr		q28, [x9, #64]
	ldr		d29, [x9, #80]
	add		x9, x9, x10
	fmla	v6.2d, v24.2d, v30.d[0]
	fmla	v7.2d, v25.2d, v30.d[0]
	fmla	v14.2d, v26.2d, v30.d[0]
	fmla	v15.2d, v27.2d, v30.d[0]
	fmla	v22.2d, v28.2d, v30.d[0]
	fmla	v23.2d, v29.2d, v30.d[0]

	b 0f

2:
	cmp		w11, #2
	blt		3f

	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	ldr		q28, [x9, #64]
	add		x9, x9, x10
	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]

	cmp		w12, #1
	ble		0f

	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	ldr		q28, [x9, #64]
	add		x9, x9, x10
	fmla	v2.2d, v24.2d, v30.d[0]
	fmla	v3.2d, v25.2d, v30.d[0]
	fmla	v10.2d, v26.2d, v30.d[0]
	fmla	v11.2d, v27.2d, v30.d[0]
	fmla	v18.2d, v28.2d, v30.d[0]

	cmp		w12, #2
	ble		0f

	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	ldr		q28, [x9, #64]
	add		x9, x9, x10
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	fmla	v12.2d, v26.2d, v30.d[0]
	fmla	v13.2d, v27.2d, v30.d[0]
	fmla	v20.2d, v28.2d, v30.d[0]

	cmp		w12, #3
	ble		0f

	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	ldr		q28, [x9, #64]
	add		x9, x9, x10
	fmla	v6.2d, v24.2d, v30.d[0]
	fmla	v7.2d, v25.2d, v30.d[0]
	fmla	v14.2d, v26.2d, v30.d[0]
	fmla	v15.2d, v27.2d, v30.d[0]
	fmla	v22.2d, v28.2d, v30.d[0]

	b 0f

3:
	cmp		w11, #1
	blt		0f

	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	ldr		d28, [x9, #64]
	add		x9, x9, x10
	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	fmla	v16.2d, v28.2d, v30.d[0]

	cmp		w12, #1
	ble		0f

	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	ldr		d28, [x9, #64]
	add		x9, x9, x10
	fmla	v2.2d, v24.2d, v30.d[0]
	fmla	v3.2d, v25.2d, v30.d[0]
	fmla	v10.2d, v26.2d, v30.d[0]
	fmla	v11.2d, v27.2d, v30.d[0]
	fmla	v18.2d, v28.2d, v30.d[0]

	cmp		w12, #2
	ble		0f

	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	ldr		d28, [x9, #64]
	add		x9, x9, x10
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	fmla	v12.2d, v26.2d, v30.d[0]
	fmla	v13.2d, v27.2d, v30.d[0]
	fmla	v20.2d, v28.2d, v30.d[0]

	cmp		w12, #3
	ble		0f

	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x9, #32]
	ldr		d28, [x9, #64]
	add		x9, x9, x10
	fmla	v6.2d, v24.2d, v30.d[0]
	fmla	v7.2d, v25.2d, v30.d[0]
	fmla	v14.2d, v26.2d, v30.d[0]
	fmla	v15.2d, v27.2d, v30.d[0]
	fmla	v22.2d, v28.2d, v30.d[0]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m1b_12x4_vs_lib)
#endif





// subroutine
//
// input arguments:
// x8  <- C
// x9  <- ldc*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M11_12X4_LIB
#else
	.align	4
	FUN_START(inner_scale_m11_12x4_lib)
#endif

	ldp		q24, q25, [x8, #0]
	ldp		q26, q27, [x8, #32]
	ldp		q28, q29, [x8, #64]
	add		x8, x8, x9
	fsub	v0.2d, v24.2d, v0.2d
	fsub	v1.2d, v25.2d, v1.2d
	fsub	v8.2d, v26.2d, v8.2d
	fsub	v9.2d, v27.2d, v9.2d
	fsub	v16.2d, v28.2d, v16.2d
	fsub	v17.2d, v29.2d, v17.2d

	ldp		q24, q25, [x8, #0]
	ldp		q26, q27, [x8, #32]
	ldp		q28, q29, [x8, #64]
	add		x8, x8, x9
	fsub	v2.2d, v24.2d, v2.2d
	fsub	v3.2d, v25.2d, v3.2d
	fsub	v10.2d, v26.2d, v10.2d
	fsub	v11.2d, v27.2d, v11.2d
	fsub	v18.2d, v28.2d, v18.2d
	fsub	v19.2d, v29.2d, v19.2d

	ldp		q24, q25, [x8, #0]
	ldp		q26, q27, [x8, #32]
	ldp		q28, q29, [x8, #64]
	add		x8, x8, x9
	fsub	v4.2d, v24.2d, v4.2d
	fsub	v5.2d, v25.2d, v5.2d
	fsub	v12.2d, v26.2d, v12.2d
	fsub	v13.2d, v27.2d, v13.2d
	fsub	v20.2d, v28.2d, v20.2d
	fsub	v21.2d, v29.2d, v21.2d

	ldp		q24, q25, [x8, #0]
	ldp		q26, q27, [x8, #32]
	ldp		q28, q29, [x8, #64]
	add		x8, x8, x9
	fsub	v6.2d, v24.2d, v6.2d
	fsub	v7.2d, v25.2d, v7.2d
	fsub	v14.2d, v26.2d, v14.2d
	fsub	v15.2d, v27.2d, v15.2d
	fsub	v22.2d, v28.2d, v22.2d
	fsub	v23.2d, v29.2d, v23.2d

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m11_12x4_lib)
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
	.macro INNER_SCALE_M11_12X4_VS_LIB
#else
	.align	4
	FUN_START(inner_scale_m11_12x4_vs_lib)
#endif

	cmp		w10, #4
	blt		1f

	ldp		q24, q25, [x8, #0]
	ldp		q26, q27, [x8, #32]
	ldp		q28, q29, [x8, #64]
	add		x8, x8, x9
	fsub	v0.2d, v24.2d, v0.2d
	fsub	v1.2d, v25.2d, v1.2d
	fsub	v8.2d, v26.2d, v8.2d
	fsub	v9.2d, v27.2d, v9.2d
	fsub	v16.2d, v28.2d, v16.2d
	fsub	v17.2d, v29.2d, v17.2d

	cmp		w11, #1
	ble		0f

	ldp		q24, q25, [x8, #0]
	ldp		q26, q27, [x8, #32]
	ldp		q28, q29, [x8, #64]
	add		x8, x8, x9
	fsub	v2.2d, v24.2d, v2.2d
	fsub	v3.2d, v25.2d, v3.2d
	fsub	v10.2d, v26.2d, v10.2d
	fsub	v11.2d, v27.2d, v11.2d
	fsub	v18.2d, v28.2d, v18.2d
	fsub	v19.2d, v29.2d, v19.2d

	cmp		w11, #2
	ble		0f

	ldp		q24, q25, [x8, #0]
	ldp		q26, q27, [x8, #32]
	ldp		q28, q29, [x8, #64]
	add		x8, x8, x9
	fsub	v4.2d, v24.2d, v4.2d
	fsub	v5.2d, v25.2d, v5.2d
	fsub	v12.2d, v26.2d, v12.2d
	fsub	v13.2d, v27.2d, v13.2d
	fsub	v20.2d, v28.2d, v20.2d
	fsub	v21.2d, v29.2d, v21.2d

	cmp		w11, #3
	ble		0f

	ldp		q24, q25, [x8, #0]
	ldp		q26, q27, [x8, #32]
	ldp		q28, q29, [x8, #64]
	add		x8, x8, x9
	fsub	v6.2d, v24.2d, v6.2d
	fsub	v7.2d, v25.2d, v7.2d
	fsub	v14.2d, v26.2d, v14.2d
	fsub	v15.2d, v27.2d, v15.2d
	fsub	v22.2d, v28.2d, v22.2d
	fsub	v23.2d, v29.2d, v23.2d

	b 0f

1:
	cmp		w10, #3
	blt		2f

	ldp		q24, q25, [x8, #0]
	ldp		q26, q27, [x8, #32]
	ldr		q28, [x8, #64]
	ldr		d29, [x8, #80]
	add		x8, x8, x9
	fsub	v0.2d, v24.2d, v0.2d
	fsub	v1.2d, v25.2d, v1.2d
	fsub	v8.2d, v26.2d, v8.2d
	fsub	v9.2d, v27.2d, v9.2d
	fsub	v16.2d, v28.2d, v16.2d
	fsub	v17.2d, v29.2d, v17.2d

	cmp		w11, #1
	ble		0f

	ldp		q24, q25, [x8, #0]
	ldp		q26, q27, [x8, #32]
	ldr		q28, [x8, #64]
	ldr		d29, [x8, #80]
	add		x8, x8, x9
	fsub	v2.2d, v24.2d, v2.2d
	fsub	v3.2d, v25.2d, v3.2d
	fsub	v10.2d, v26.2d, v10.2d
	fsub	v11.2d, v27.2d, v11.2d
	fsub	v18.2d, v28.2d, v18.2d
	fsub	v19.2d, v29.2d, v19.2d

	cmp		w11, #2
	ble		0f

	ldp		q24, q25, [x8, #0]
	ldp		q26, q27, [x8, #32]
	ldr		q28, [x8, #64]
	ldr		d29, [x8, #80]
	add		x8, x8, x9
	fsub	v4.2d, v24.2d, v4.2d
	fsub	v5.2d, v25.2d, v5.2d
	fsub	v12.2d, v26.2d, v12.2d
	fsub	v13.2d, v27.2d, v13.2d
	fsub	v20.2d, v28.2d, v20.2d
	fsub	v21.2d, v29.2d, v21.2d

	cmp		w11, #3
	ble		0f

	ldp		q24, q25, [x8, #0]
	ldp		q26, q27, [x8, #32]
	ldr		q28, [x8, #64]
	ldr		d29, [x8, #80]
	add		x8, x8, x9
	fsub	v6.2d, v24.2d, v6.2d
	fsub	v7.2d, v25.2d, v7.2d
	fsub	v14.2d, v26.2d, v14.2d
	fsub	v15.2d, v27.2d, v15.2d
	fsub	v22.2d, v28.2d, v22.2d
	fsub	v23.2d, v29.2d, v23.2d

	b 0f

2:
	cmp		w10, #2
	blt		3f

	ldp		q24, q25, [x8, #0]
	ldp		q26, q27, [x8, #32]
	ldr		q28, [x8, #64]
	add		x8, x8, x9
	fsub	v0.2d, v24.2d, v0.2d
	fsub	v1.2d, v25.2d, v1.2d
	fsub	v8.2d, v26.2d, v8.2d
	fsub	v9.2d, v27.2d, v9.2d
	fsub	v16.2d, v28.2d, v16.2d

	cmp		w11, #1
	ble		0f

	ldp		q24, q25, [x8, #0]
	ldp		q26, q27, [x8, #32]
	ldr		q28, [x8, #64]
	add		x8, x8, x9
	fsub	v2.2d, v24.2d, v2.2d
	fsub	v3.2d, v25.2d, v3.2d
	fsub	v10.2d, v26.2d, v10.2d
	fsub	v11.2d, v27.2d, v11.2d
	fsub	v18.2d, v28.2d, v18.2d

	cmp		w11, #2
	ble		0f

	ldp		q24, q25, [x8, #0]
	ldp		q26, q27, [x8, #32]
	ldr		q28, [x8, #64]
	add		x8, x8, x9
	fsub	v4.2d, v24.2d, v4.2d
	fsub	v5.2d, v25.2d, v5.2d
	fsub	v12.2d, v26.2d, v12.2d
	fsub	v13.2d, v27.2d, v13.2d
	fsub	v20.2d, v28.2d, v20.2d

	cmp		w11, #3
	ble		0f

	ldp		q24, q25, [x8, #0]
	ldp		q26, q27, [x8, #32]
	ldr		q28, [x8, #64]
	add		x8, x8, x9
	fsub	v6.2d, v24.2d, v6.2d
	fsub	v7.2d, v25.2d, v7.2d
	fsub	v14.2d, v26.2d, v14.2d
	fsub	v15.2d, v27.2d, v15.2d
	fsub	v22.2d, v28.2d, v22.2d

	b 0f

3:
	cmp		w10, #1
	blt		0f

	ldp		q24, q25, [x8, #0]
	ldp		q26, q27, [x8, #32]
	ldr		d28, [x8, #64]
	add		x8, x8, x9
	fsub	v0.2d, v24.2d, v0.2d
	fsub	v1.2d, v25.2d, v1.2d
	fsub	v8.2d, v26.2d, v8.2d
	fsub	v9.2d, v27.2d, v9.2d
	fsub	v16.2d, v28.2d, v16.2d

	cmp		w11, #1
	ble		0f

	ldp		q24, q25, [x8, #0]
	ldp		q26, q27, [x8, #32]
	ldr		d28, [x8, #64]
	add		x8, x8, x9
	fsub	v2.2d, v24.2d, v2.2d
	fsub	v3.2d, v25.2d, v3.2d
	fsub	v10.2d, v26.2d, v10.2d
	fsub	v11.2d, v27.2d, v11.2d
	fsub	v18.2d, v28.2d, v18.2d

	cmp		w11, #2
	ble		0f

	ldp		q24, q25, [x8, #0]
	ldp		q26, q27, [x8, #32]
	ldr		d28, [x8, #64]
	add		x8, x8, x9
	fsub	v4.2d, v24.2d, v4.2d
	fsub	v5.2d, v25.2d, v5.2d
	fsub	v12.2d, v26.2d, v12.2d
	fsub	v13.2d, v27.2d, v13.2d
	fsub	v20.2d, v28.2d, v20.2d

	cmp		w11, #3
	ble		0f

	ldp		q24, q25, [x8, #0]
	ldp		q26, q27, [x8, #32]
	ldr		d28, [x8, #64]
	add		x8, x8, x9
	fsub	v6.2d, v24.2d, v6.2d
	fsub	v7.2d, v25.2d, v7.2d
	fsub	v14.2d, v26.2d, v14.2d
	fsub	v15.2d, v27.2d, v15.2d
	fsub	v22.2d, v28.2d, v22.2d

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m11_12x4_vs_lib)
#endif





// subroutine
//
// input arguments:
// x8   <- D
// x9   <- ldd*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_12X4_LIB
#else
	.align 4
	FUN_START(inner_store_12x4_lib)
#endif

	stp		q0, q1, [x8, #0]
	stp		q8, q9, [x8, #32]
	stp		q16, q17, [x8, #64]
	add		x8, x8, x9
	stp		q2, q3, [x8, #0]
	stp		q10, q11, [x8, #32]
	stp		q18, q19, [x8, #64]
	add		x8, x8, x9
	stp		q4, q5, [x8, #0]
	stp		q12, q13, [x8, #32]
	stp		q20, q21, [x8, #64]
	add		x8, x8, x9
	stp		q6, q7, [x8, #0]
	stp		q14, q15, [x8, #32]
	stp		q22, q23, [x8, #64]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_12x4_lib)
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
	.macro INNER_STORE_12X4_VS_LIB
#else
	.align 4
	FUN_START(inner_store_12x4_vs_lib)
#endif

	cmp		w10, #12
	blt		1f

	// 1st col
	stp		q0, q1, [x8, #0]
	stp		q8, q9, [x8, #32]
	stp		q16, q17, [x8, #64]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	stp		q2, q3, [x8, #0]
	stp		q10, q11, [x8, #32]
	stp		q18, q19, [x8, #64]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	stp		q4, q5, [x8, #0]
	stp		q12, q13, [x8, #32]
	stp		q20, q21, [x8, #64]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	stp		q6, q7, [x8, #0]
	stp		q14, q15, [x8, #32]
	stp		q22, q23, [x8, #64]
	b		0f

1:
	cmp		w10, #11
	blt		1f

	// 1st col
	stp		q0, q1, [x8, #0]
	stp		q8, q9, [x8, #32]
	str		q16, [x8, #64]
	str		d17, [x8, #80]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	stp		q2, q3, [x8, #0]
	stp		q10, q11, [x8, #32]
	str		q18, [x8, #64]
	str		d19, [x8, #80]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	stp		q4, q5, [x8, #0]
	stp		q12, q13, [x8, #32]
	str		q20, [x8, #64]
	str		d21, [x8, #80]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	stp		q6, q7, [x8, #0]
	stp		q14, q15, [x8, #32]
	str		q22, [x8, #64]
	str		d23, [x8, #80]
	b		0f

1:
	cmp		w10, #10
	blt		1f

	// 1st col
	stp		q0, q1, [x8, #0]
	stp		q8, q9, [x8, #32]
	str		q16, [x8, #64]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	stp		q2, q3, [x8, #0]
	stp		q10, q11, [x8, #32]
	str		q18, [x8, #64]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	stp		q4, q5, [x8, #0]
	stp		q12, q13, [x8, #32]
	str		q20, [x8, #64]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	stp		q6, q7, [x8, #0]
	stp		q14, q15, [x8, #32]
	str		q22, [x8, #64]
	b		0f

1:
//	cmp		w10, #9
//	blt		0f

	// 1st col
	stp		q0, q1, [x8, #0]
	stp		q8, q9, [x8, #32]
	str		d16, [x8, #64]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	stp		q2, q3, [x8, #0]
	stp		q10, q11, [x8, #32]
	str		d18, [x8, #64]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	stp		q4, q5, [x8, #0]
	stp		q12, q13, [x8, #32]
	str		d20, [x8, #64]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	stp		q6, q7, [x8, #0]
	stp		q14, q15, [x8, #32]
	str		d22, [x8, #64]
//	b		0f

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_12x4_vs_lib)
#endif





// subroutine
//
// input arguments:
// x8   <- D
// x9   <- ldd*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_12X4_LIB
#else
	.align 4
	FUN_START(inner_store_l_12x4_lib)
#endif

	ins		v24.d[0], v2.d[1]
	ins		v25.d[0], v7.d[1]

	stp		q0, q1, [x8, #0]
	stp		q8, q9, [x8, #32]
	stp		q16, q17, [x8, #64]
	add		x8, x8, x9
	str		d24, [x8, #8]
	str		q3, [x8, #16]
	stp		q10, q11, [x8, #32]
	stp		q18, q19, [x8, #64]
	add		x8, x8, x9
	str		q5, [x8, #16]
	stp		q12, q13, [x8, #32]
	stp		q20, q21, [x8, #64]
	add		x8, x8, x9
	str		d25, [x8, #24]
	stp		q14, q15, [x8, #32]
	stp		q22, q23, [x8, #64]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_12x4_lib)
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
	.macro INNER_STORE_L_12X4_VS_LIB
#else
	.align 4
	FUN_START(inner_store_l_12x4_vs_lib)
#endif

	cmp		w10, #12
	blt		1f

	// 1st col
	stp		q0, q1, [x8, #0]
	stp		q8, q9, [x8, #32]
	stp		q16, q17, [x8, #64]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	ins		v24.d[0], v2.d[1]
	str		d24, [x8, #8]
	str		q3, [x8, #16]
	stp		q10, q11, [x8, #32]
	stp		q18, q19, [x8, #64]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	str		q5, [x8, #16]
	stp		q12, q13, [x8, #32]
	stp		q20, q21, [x8, #64]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	ins		v25.d[0], v7.d[1]
	str		d25, [x8, #24]
	stp		q14, q15, [x8, #32]
	stp		q22, q23, [x8, #64]
	b		0f

1:
	cmp		w10, #11
	blt		1f

	// 1st col
	stp		q0, q1, [x8, #0]
	stp		q8, q9, [x8, #32]
	str		q16, [x8, #64]
	str		d17, [x8, #80]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	ins		v24.d[0], v2.d[1]
	str		d24, [x8, #8]
	str		q3, [x8, #16]
	stp		q10, q11, [x8, #32]
	str		q18, [x8, #64]
	str		d19, [x8, #80]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	str		q5, [x8, #16]
	stp		q12, q13, [x8, #32]
	str		q20, [x8, #64]
	str		d21, [x8, #80]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	ins		v25.d[0], v7.d[1]
	str		d25, [x8, #24]
	stp		q14, q15, [x8, #32]
	str		q22, [x8, #64]
	str		d23, [x8, #80]
	b		0f

1:
	cmp		w10, #10
	blt		1f

	// 1st col
	stp		q0, q1, [x8, #0]
	stp		q8, q9, [x8, #32]
	str		q16, [x8, #64]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	ins		v24.d[0], v2.d[1]
	str		d24, [x8, #8]
	str		q3, [x8, #16]
	stp		q10, q11, [x8, #32]
	str		q18, [x8, #64]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	str		q5, [x8, #16]
	stp		q12, q13, [x8, #32]
	str		q20, [x8, #64]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	ins		v25.d[0], v7.d[1]
	str		d25, [x8, #24]
	stp		q14, q15, [x8, #32]
	str		q22, [x8, #64]
	b		0f

1:
//	cmp		w10, #9
//	blt		0f

	// 1st col
	stp		q0, q1, [x8, #0]
	stp		q8, q9, [x8, #32]
	str		d16, [x8, #64]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	ins		v24.d[0], v2.d[1]
	str		d24, [x8, #8]
	str		q3, [x8, #16]
	stp		q10, q11, [x8, #32]
	str		d18, [x8, #64]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	str		q5, [x8, #16]
	stp		q12, q13, [x8, #32]
	str		d20, [x8, #64]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	ins		v25.d[0], v7.d[1]
	str		d25, [x8, #24]
	stp		q14, q15, [x8, #32]
	str		d22, [x8, #64]
//	b		0f

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_12x4_vs_lib)
#endif





// subroutine
//
// input arguments:
// x8   <- D
// x9   <- ldd*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_U_12X4_LIB
#else
	.align 4
	FUN_START(inner_store_u_12x4_lib)
#endif

	stp		q0, q1, [x8, #0]
	stp		q8, q9, [x8, #32]
	str		d16, [x8, #64]
	add		x8, x8, x9
	stp		q2, q3, [x8, #0]
	stp		q10, q11, [x8, #32]
	str		q18, [x8, #64]
	add		x8, x8, x9
	stp		q4, q5, [x8, #0]
	stp		q12, q13, [x8, #32]
	str		q20, [x8, #64]
	str		d21, [x8, #80]
	add		x8, x8, x9
	stp		q6, q7, [x8, #0]
	stp		q14, q15, [x8, #32]
	stp		q22, q23, [x8, #64]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_u_12x4_lib)
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
	.macro INNER_STORE_U_12X4_VS_LIB
#else
	.align 4
	FUN_START(inner_store_u_12x4_vs_lib)
#endif

	cmp		w10, #12
	blt		1f

	// 1st col
	stp		q0, q1, [x8, #0]
	stp		q8, q9, [x8, #32]
	str		d16, [x8, #64]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	stp		q2, q3, [x8, #0]
	stp		q10, q11, [x8, #32]
	str		q18, [x8, #64]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	stp		q4, q5, [x8, #0]
	stp		q12, q13, [x8, #32]
	str		q20, [x8, #64]
	str		d21, [x8, #80]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	stp		q6, q7, [x8, #0]
	stp		q14, q15, [x8, #32]
	stp		q22, q23, [x8, #64]
	b		0f

1:
	cmp		w10, #11
	blt		1f

	// 1st col
	stp		q0, q1, [x8, #0]
	stp		q8, q9, [x8, #32]
	str		d16, [x8, #64]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	stp		q2, q3, [x8, #0]
	stp		q10, q11, [x8, #32]
	str		q18, [x8, #64]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	stp		q4, q5, [x8, #0]
	stp		q12, q13, [x8, #32]
	str		q20, [x8, #64]
	str		d21, [x8, #80]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	stp		q6, q7, [x8, #0]
	stp		q14, q15, [x8, #32]
	str		q22, [x8, #64]
	str		d23, [x8, #80]
	b		0f

1:
	cmp		w10, #10
	blt		1f

	// 1st col
	stp		q0, q1, [x8, #0]
	stp		q8, q9, [x8, #32]
	str		d16, [x8, #64]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	stp		q2, q3, [x8, #0]
	stp		q10, q11, [x8, #32]
	str		q18, [x8, #64]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	stp		q4, q5, [x8, #0]
	stp		q12, q13, [x8, #32]
	str		q20, [x8, #64]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	stp		q6, q7, [x8, #0]
	stp		q14, q15, [x8, #32]
	str		q22, [x8, #64]
	b		0f

1:
//	cmp		w10, #9
//	blt		0f

	// 1st col
	stp		q0, q1, [x8, #0]
	stp		q8, q9, [x8, #32]
	str		d16, [x8, #64]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	stp		q2, q3, [x8, #0]
	stp		q10, q11, [x8, #32]
	str		d18, [x8, #64]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	stp		q4, q5, [x8, #0]
	stp		q12, q13, [x8, #32]
	str		d20, [x8, #64]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	stp		q6, q7, [x8, #0]
	stp		q14, q15, [x8, #32]
	str		d22, [x8, #64]
//	b		0f

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_u_12x4_vs_lib)
#endif





// subroutine
//
// input arguments:
// x8   <- D
// x9   <- ldd*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_PREFETCH_12X4_LIB
#else
	.align 4
	FUN_START(inner_prefetch_12x4_lib)
#endif

	prfm	PLDL1KEEP, [x8, #0]
	prfm	PLDL1KEEP, [x8, #64]
	add		x8, x8, x9
	prfm	PLDL1KEEP, [x8, #0]
	prfm	PLDL1KEEP, [x8, #64]
	add		x8, x8, x9
	prfm	PLDL1KEEP, [x8, #0]
	prfm	PLDL1KEEP, [x8, #64]
	add		x8, x8, x9
	prfm	PLDL1KEEP, [x8, #0]
	prfm	PLDL1KEEP, [x8, #64]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_prefetch_12x4_lib)
#endif





//                                  w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8
// void kernel_dgemm_nt_12x4_lib44cc(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dgemm_nt_12x4_lib44cc)
	FUN_START(kernel_dgemm_nt_12x4_lib44cc)
	


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
	INNER_PREFETCH_12X4_LIB
#else
	CALL(inner_prefetch_12x4_lib)
#endif

100:


	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_12x4_lib4)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_12X4_LIB
#else
	CALL(inner_prefetch_12x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // ldc
	lsl		w11, w11, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nt_12x4_lib44cc)





//                                     w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8     sp+16   sp+24
// void kernel_dgemm_nt_12x4_vs_lib44cc(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dgemm_nt_12x4_vs_lib44cc)
	FUN_START(kernel_dgemm_nt_12x4_vs_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_12x4_lib4)
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
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd
	ldr		w10, [sp, #(STACKSIZE + 16)] // m1
	ldr		w11, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nt_12x4_vs_lib44cc)





//                                  w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16
// void kernel_dgemm_nn_12x4_lib4ccc(int kmax, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dgemm_nn_12x4_lib4ccc)
	FUN_START(kernel_dgemm_nn_12x4_lib4ccc)
	


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
	INNER_KERNEL_GEMM_ADD_NN_12X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_12x4_lib4c)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_12X4_LIB
#else
	CALL(inner_prefetch_12x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // ldc
	lsl		w11, w11, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nn_12x4_lib4ccc)





//                                     w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16    sp+24   sp+32
// void kernel_dgemm_nn_12x4_vs_lib4ccc(int kmax, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dgemm_nn_12x4_vs_lib4ccc)
	FUN_START(kernel_dgemm_nn_12x4_vs_lib4ccc)
	


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
	INNER_KERNEL_GEMM_ADD_NN_12X1_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_12x1_lib4c)
#endif

	b		103f

100:

	ldr		w13, [sp, #(STACKSIZE + 32)] // n1
	cmp		w13, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_12X2_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_12x2_lib4c)
#endif
	
	b		103f

101:

	ldr		w13, [sp, #(STACKSIZE + 32)] // n1
	cmp		w13, #3
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_12X3_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_12x3_lib4c)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_12X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_12x4_lib4c)
#endif

103:



	// prefetch
	// TODO prefethc vs
//	ldr		x8, [sp, #(STACKSIZE + 8)] // D
//	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
//	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
//	INNER_PREFETCH_12X4_LIB
#else
//	CALL(inner_prefetch_12x4_lib)
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
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*sdd
	ldr		w10, [sp, #(STACKSIZE + 24)] // m1
	ldr		w11, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nn_12x4_vs_lib4ccc)





//                                    w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8
// void kernel_dsyrk_nt_l_12x4_lib44cc(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dsyrk_nt_l_12x4_lib44cc)
	FUN_START(kernel_dsyrk_nt_l_12x4_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_12x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // ldc
	lsl		w11, w11, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_L_12X4_LIB
#else
	CALL(inner_store_l_12x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dsyrk_nt_l_12x4_lib44cc)





//                                       w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8     sp+16   sp+24
// void kernel_dsyrk_nt_l_12x4_vs_lib44cc(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dsyrk_nt_l_12x4_vs_lib44cc)
	FUN_START(kernel_dsyrk_nt_l_12x4_vs_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_12x4_lib4)
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
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd
	ldr		w10, [sp, #(STACKSIZE + 16)] // m1
	ldr		w11, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_12X4_VS_LIB
#else
	CALL(inner_store_l_12x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dsyrk_nt_l_12x4_vs_lib44cc)





//                                    w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8
// void kernel_dsyrk_nt_u_12x4_lib44cc(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dsyrk_nt_u_12x4_lib44cc)
	FUN_START(kernel_dsyrk_nt_u_12x4_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_12x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // ldc
	lsl		w11, w11, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_U_12X4_LIB
#else
	CALL(inner_store_u_12x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dsyrk_nt_u_12x4_lib44cc)





//                                       w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8     sp+16   sp+24
// void kernel_dsyrk_nt_u_12x4_vs_lib44cc(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dsyrk_nt_u_12x4_vs_lib44cc)
	FUN_START(kernel_dsyrk_nt_u_12x4_vs_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_12x4_lib4)
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
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd
	ldr		w10, [sp, #(STACKSIZE + 16)] // m1
	ldr		w11, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_U_12X4_VS_LIB
#else
	CALL(inner_store_u_12x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dsyrk_nt_u_12x4_vs_lib44cc)





//                                          w0        x1         w2       x3         x4         w5       x6         w7       sp+0       sp+8     sp+16
// void kernel_dtrsm_nt_rl_inv_12x4_lib44ccc(int kmax, double *A, int sda, double *B, double *C, int ldc, double *D, int ldd, double *E, int lde, double *inv_diag_E)

	.align	4
	GLOB(kernel_dtrsm_nt_rl_inv_12x4_lib44ccc)
	FUN_START(kernel_dtrsm_nt_rl_inv_12x4_lib44ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_12x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x4 // C
	mov		w9, w5 // ldc
	lsl		w9, w9, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB
#else
	CALL(inner_scale_m11_12x4_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 0)] // E
	ldr		w9, [sp, #(STACKSIZE + 8)] // sde
	lsl		w9, w9, #3 // 8*ldc
	ldr		x10, [sp, #(STACKSIZE + 16)] // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_12X4_LIB
#else
	CALL(inner_edge_trsm_rlt_inv_12x4_lib)
#endif



	// store l
	mov		x8, x6 // D
	mov		w9, w7 // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_12x4_lib44ccc)





//                                             w0        x1         w2       x3         x4         w5       x6         w7       sp+0       sp+8     sp+16               sp+24   sp+32
// void kernel_dtrsm_nt_rl_inv_12x4_vs_lib44ccc(int kmax, double *A, int sda, double *B, double *C, int ldc, double *D, int ldd, double *E, int lde, double *inv_diag_E, int m1, int n1)

	.align	4
	GLOB(kernel_dtrsm_nt_rl_inv_12x4_vs_lib44ccc)
	FUN_START(kernel_dtrsm_nt_rl_inv_12x4_vs_lib44ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_12x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x4 // C
	mov		w9, w5 // ldc
	lsl		w9, w9, #3 // 8*ldc
	ldr		w10, [sp, #(STACKSIZE + 24)] // m1
	ldr		w11, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_VS_LIB
#else
	CALL(inner_scale_m11_12x4_vs_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 0)] // E
	ldr		w9, [sp, #(STACKSIZE + 8)] // sde
	lsl		w9, w9, #3 // 8*ldc
	ldr		x10, [sp, #(STACKSIZE + 16)] // inv_diag_E
	ldr		w11, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_12X4_VS_LIB
#else
	CALL(inner_edge_trsm_rlt_inv_12x4_vs_lib)
#endif



	// store l
	mov		x8, x6 // D
	mov		w9, w7 // ldd
	lsl		w9, w9, #3 // 8*ldd
	ldr		w10, [sp, #(STACKSIZE + 24)] // m1
	ldr		w11, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_12x4_vs_lib44ccc)





//                                          w0        x1         w2       x3         x4            x5         w6       x7         sp+0     sp+8       sp+16
// void kernel_dtrsm_nt_rl_inv_12x4_lib44cc4(int kmax, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E, double *inv_diag_E)

	.align	4
	GLOB(kernel_dtrsm_nt_rl_inv_12x4_lib44cc4)
	FUN_START(kernel_dtrsm_nt_rl_inv_12x4_lib44cc4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_12x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta
	mov		x8, x4 // beta
	mov		x9, x5 // C
	mov		w10, w6 // ldc
	lsl		w10, w10, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB
#else
	CALL(inner_scale_m1b_12x4_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 8)] // E
	ldr		x9, [sp, #(STACKSIZE + 16)] // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_12X4_LIB4
#else
	CALL(inner_edge_trsm_rlt_inv_12x4_lib4)
#endif



	// store l
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_12x4_lib44cc4)





//                                             w0        x1         w2       x3         x4            x5         w6       x7         sp+0     sp+8       sp+16               sp+24   sp+32
// void kernel_dtrsm_nt_rl_inv_12x4_vs_lib44cc4(int kmax, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E, double *inv_diag_E, int m1, int n1)

	.align	4
	GLOB(kernel_dtrsm_nt_rl_inv_12x4_vs_lib44cc4)
	FUN_START(kernel_dtrsm_nt_rl_inv_12x4_vs_lib44cc4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_12x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x4 // beta
	mov		x9, x5 // C
	mov		w10, w6 // ldc
	lsl		w10, w10, #3 // 8*ldc
	ldr		w11, [sp, #(STACKSIZE + 24)] // m1
	ldr		w12, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_VS_LIB
#else
	CALL(inner_scale_m1b_12x4_vs_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 8)] // E
	ldr		x9, [sp, #(STACKSIZE + 16)] // inv_diag_E
	ldr		w10, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_12X4_VS_LIB4
#else
	CALL(inner_edge_trsm_rlt_inv_12x4_vs_lib4)
#endif



	// store l
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w9, w9, #3 // 8*ldd
	ldr		w10, [sp, #(STACKSIZE + 24)] // m1
	ldr		w11, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_12x4_vs_lib44cc4)





//                                          w0        x1         w2       x3         x4            x5         w6       x7         sp+0     sp+8
// void kernel_dtrsm_nt_rl_one_12x4_lib44cc4(int kmax, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E)

	.align	4
	GLOB(kernel_dtrsm_nt_rl_one_12x4_lib44cc4)
	FUN_START(kernel_dtrsm_nt_rl_one_12x4_lib44cc4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_12x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x4 // beta
	mov		x9, x5 // C
	mov		w10, w6 // ldc
	lsl		w10, w10, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB
#else
	CALL(inner_scale_m1b_12x4_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 8)] // E

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_ONE_12X4_LIB4
#else
	CALL(inner_edge_trsm_rlt_one_12x4_lib4)
#endif



	// store l
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nt_rl_one_12x4_lib44cc4)





//                                          w0        x1         w2       x3         x4            x5         w6       x7         sp+0     sp+8       sp+16      sp+24
// void kernel_dtrsm_nt_rl_one_12x4_vs_lib44cc4(int kmax, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E, int m1, int n1)

	.align	4
	GLOB(kernel_dtrsm_nt_rl_one_12x4_vs_lib44cc4)
	FUN_START(kernel_dtrsm_nt_rl_one_12x4_vs_lib44cc4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_12x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x4 // beta
	mov		x9, x5 // C
	mov		w10, w6 // ldc
	lsl		w10, w10, #3 // 8*ldc
	ldr		w11, [sp, #(STACKSIZE + 16)] // m1
	ldr		w12, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_VS_LIB
#else
	CALL(inner_scale_m1b_12x4_vs_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 8)] // E
	ldr		w9, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_ONE_12X4_VS_LIB4
#else
	CALL(inner_edge_trsm_rlt_one_12x4_vs_lib4)
#endif



	// store l
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w9, w9, #3 // 8*ldd
	ldr		w10, [sp, #(STACKSIZE + 16)] // m1
	ldr		w11, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nt_rl_one_12x4_vs_lib44cc4)





//                                     w0        x1         w2       x3         x4         w5       x6         w7       sp+0
// void kernel_dpotrf_nt_l_12x4_lib44cc(int kmax, double *A, int sda, double *B, double *C, int ldc, double *D, int ldd, double *inv_diag_D)

	.align	4
	GLOB(kernel_dpotrf_nt_l_12x4_lib44cc)
	FUN_START(kernel_dpotrf_nt_l_12x4_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_12x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x4 // C
	mov		w9, w5 // ldc
	lsl		w9, w9, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB
#else
	CALL(inner_scale_m11_12x4_lib)
#endif



	// factorization
	ldr		x8, [sp, #(STACKSIZE + 0)] // inv_diag_D

#if MACRO_LEVEL>=1
	INNER_EDGE_POTRF_12X4_LIB4
#else
	CALL(inner_edge_potrf_12x4_lib4)
#endif



	// store l
	mov		x8, x6 // D
	mov		w9, w7 // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_L_12X4_LIB
#else
	CALL(inner_store_l_12x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dpotrf_nt_l_12x4_lib44cc)





//                                        w0        x1         w2       x3         x4         xw       x6         w7       sp+0                sp+8,   sp+16
// void kernel_dpotrf_nt_l_12x4_vs_lib44cc(int kmax, double *A, int sda, double *B, double *C, int ldc, double *D, int ldd, double *inv_diag_D, int m1, int n1)

	.align	4
	GLOB(kernel_dpotrf_nt_l_12x4_vs_lib44cc)
	FUN_START(kernel_dpotrf_nt_l_12x4_vs_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_12x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x4 // C
	mov		w9, w5 // ldc
	lsl		w9, w9, #3 // 8*ldc
	ldr		w10, [sp, #(STACKSIZE + 8)] // m1
	ldr		w11, [sp, #(STACKSIZE + 16)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_VS_LIB
#else
	CALL(inner_scale_m11_12x4_vs_lib)
#endif



	// factorization
	ldr		x8, [sp, #(STACKSIZE + 0)] // inv_diag_D
	ldr		w9, [sp, #(STACKSIZE + 16)] // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_POTRF_12X4_VS_LIB4
#else
	CALL(inner_edge_potrf_12x4_vs_lib4)
#endif



	// store l
	mov		x8, x6 // D
	mov		w9, w7 // ldd
	lsl		w9, w9, #3 // 8*ldd
	ldr		w10, [sp, #(STACKSIZE + 8)] // m1
	ldr		w11, [sp, #(STACKSIZE + 16)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_12X4_VS_LIB
#else
	CALL(inner_store_l_12x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dpotrf_nt_l_12x4_vs_lib44cc)





//                                          w0        x1         w2       x3         w4       x5            x6         w7       sp+0       sp+8     sp+16      sp+24
// void kernel_dtrsm_nn_ll_one_12x4_lib4cccc(int kmax, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde)

	.align	4
	GLOB(kernel_dtrsm_nn_ll_one_12x4_lib4cccc)
	FUN_START(kernel_dtrsm_nn_ll_one_12x4_lib4cccc)
	


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
	INNER_KERNEL_GEMM_ADD_NN_12X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_12x4_lib4c)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x5 // beta
	mov		x9, x6 // C
	mov		w10, w7 // ldc
	lsl		w10, w10, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB
#else
	CALL(inner_scale_m1b_12x4_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 16)] // E
	ldr		w9, [sp, #(STACKSIZE + 24)] // sde
	lsl		w9, w9, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_LLN_ONE_12X4_LIB
#else
	CALL(inner_edge_trsm_lln_one_12x4_lib)
#endif



	// store l
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // sdd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nn_ll_one_12x4_lib4cccc)







//                                             w0        x1         w2       x3         w4       x5            x6         w7       sp+0       sp+8     sp+16      sp+24    sp+32   sp+40
// void kernel_dtrsm_nn_ll_one_12x4_vs_lib4cccc(int kmax, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, int m1, int n1)

	.align	4
	GLOB(kernel_dtrsm_nn_ll_one_12x4_vs_lib4cccc)
	FUN_START(kernel_dtrsm_nn_ll_one_12x4_vs_lib4cccc)
	


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
	INNER_KERNEL_GEMM_ADD_NN_12X1_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_12x1_lib4c)
#endif

	b		103f

100:

	ldr		w13, [sp, #(STACKSIZE + 40)] // n1
	cmp		w13, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_12X2_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_12x2_lib4c)
#endif
	
	b		103f

101:

	ldr		w13, [sp, #(STACKSIZE + 40)] // n1
	cmp		w13, #3
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_12X3_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_12x3_lib4c)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_12X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_12x4_lib4c)
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
	INNER_SCALE_M1B_12X4_VS_LIB
#else
	CALL(inner_scale_m1b_12x4_vs_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 16)] // E
	ldr		w9, [sp, #(STACKSIZE + 24)] // sde
	lsl		w9, w9, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_LLN_ONE_12X4_LIB
#else
	CALL(inner_edge_trsm_lln_one_12x4_lib)
#endif



	// store l
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // sdd
	lsl		w9, w9, #3 // 8*ldd
	ldr		w10, [sp, #(STACKSIZE + 32)] // m1
	ldr		w11, [sp, #(STACKSIZE + 40)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nn_ll_one_12x4_vs_lib4cccc)







