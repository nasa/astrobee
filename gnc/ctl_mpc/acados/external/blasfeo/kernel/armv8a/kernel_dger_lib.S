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
// x9   <- x
// x10  <- C
// w11  <- ldc*sizeof(double)
// x12  <- C
// w13  <- ldc*sizeof(double)
// v0   <- [y_0, y_1]
// v1   <- [y_2, y_3]
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GER_4_LIBC
#else
	.align	4
	FUN_START(inner_kernel_ger_4_libc)
#endif

	// early return
	cmp		w8, #0
	ble		2f // return

	add		x14, x10, x11
	add		x15, x10, x11, lsl #1
	add		x16, x14, x11, lsl #1

	add		x17, x12, x13
	add		x18, x12, x13, lsl #1
	add		x19, x17, x13, lsl #1

	cmp		w8, #4
	ble		0f // consider clean up loop

	// main loop
1:
	
	ldp		q8, q9, [x9], #32

	ldp		q16, q17, [x10], #32
	fmla	v16.2d, v8.2d, v0.d[0]
	fmla	v17.2d, v9.2d, v0.d[0]

	ldp		q18, q19, [x14], #32
	fmla	v18.2d, v8.2d, v0.d[1]
	fmla	v19.2d, v9.2d, v0.d[1]

	ldp		q20, q21, [x15], #32
	fmla	v20.2d, v8.2d, v1.d[0]
	fmla	v21.2d, v9.2d, v1.d[0]

	ldp		q22, q23, [x16], #32
	fmla	v22.2d, v8.2d, v1.d[1]
	fmla	v23.2d, v9.2d, v1.d[1]

	sub		w8, w8, #4

	stp		q16, q17, [x12], #32
	stp		q18, q19, [x17], #32
	stp		q20, q21, [x18], #32
	stp		q22, q23, [x19], #32

	cmp		w8, #4
	bgt		1b

0:

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		d8, [x9], #8

	ldr		d16, [x10], #8
	fmla	v16.2d, v8.2d, v0.d[0]

	ldr		d18, [x14], #8
	fmla	v18.2d, v8.2d, v0.d[1]

	ldr		d20, [x15], #8
	fmla	v20.2d, v8.2d, v1.d[0]

	ldr		d22, [x16], #8
	fmla	v22.2d, v8.2d, v1.d[1]

	sub		w8, w8, #1

	str		d16, [x12], #8
	str		d18, [x17], #8
	str		d20, [x18], #8
	str		d22, [x19], #8

	cmp		w8, #0
	bgt		3b

2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_ger_4_libc)
#endif





//                       w0        x1             x2         x3         x4         w5       x6         w7
// void kernel_dger_libc(int kmax, double *alpha, double *x, double *y, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dger_4_libc)
	FUN_START(kernel_dger_4_libc)
	

	PROLOGUE


	// initialize n
	ldr		d16, [x1] // alpha
	ldp		q0, q1, [x3] // y
	fmul	v0.2d, v0.2d, v16.d[0]
	fmul	v1.2d, v1.2d, v16.d[0]


	// call inner kernel gemv n
	mov		w8, w0 // kmax
	mov		x9, x2 // x
	mov		x10, x4 // C
	mov		w11, w5 // ldc
	lsl		w11, w11, #3 // 8*ldc
	mov		x12, x6 // D
	mov		w13, w7 // ldd
	lsl		w13, w13, #3 // 8*ldd

#if MACRO_LEVEL>=2
	INNER_KERNEL_GER_4_LIBC
#else
	CALL(inner_kernel_ger_4_libc)
#endif


	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dger_4_libc)






