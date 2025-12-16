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

#if defined(OS_LINUX)

#define STACKSIZE 11*16
#define PROLOGUE \
	sub sp, sp, #(11 * 16); \
	stp d8, d9, [sp, #(0 * 16)]; \
	stp d10, d11, [sp, #(1 * 16)]; \
	stp d12, d13, [sp, #(2 * 16)]; \
	stp d14, d15, [sp, #(3 * 16)]; \
	stp x18, x19, [sp, #(4 * 16)]; \
	stp x20, x21, [sp, #(5 * 16)]; \
	stp x22, x23, [sp, #(6 * 16)]; \
	stp x24, x25, [sp, #(7 * 16)]; \
	stp x26, x27, [sp, #(8 * 16)]; \
	stp x28, x29, [sp, #(9 * 16)]; \
	str x30, [sp, #(10 * 16)];
#define EPILOGUE \
	ldp d8, d9, [sp, #(0 * 16)]; \
	ldp d10, d11, [sp, #(1 * 16)]; \
	ldp d12, d13, [sp, #(2 * 16)]; \
	ldp d14, d15, [sp, #(3 * 16)]; \
	ldp x18, x19, [sp, #(4 * 16)]; \
	ldp x20, x21, [sp, #(5 * 16)]; \
	ldp x22, x23, [sp, #(6 * 16)]; \
	ldp x24, x25, [sp, #(7 * 16)]; \
	ldp x26, x27, [sp, #(8 * 16)]; \
	ldp x28, x29, [sp, #(9 * 16)]; \
	ldr x30, [sp, #(10 * 16)]; \
	add sp, sp, #(11 * 16);
#define GLOB(NAME) \
	.global	NAME
#define FUN_START(NAME) \
	.type NAME, %function; \
NAME:
#define FUN_END(NAME) \
	.size	NAME, .-NAME
#define CALL(NAME) \
	bl NAME
#define ZERO_ACC \
	fmov	d0, xzr; \
	fmov    d1, d0; \
	fmov    d2, d0; \
	fmov    d3, d0; \
	fmov    d4, d0; \
	fmov    d5, d0; \
	fmov    d6, d0; \
	fmov    d7, d0; \
	fmov    d8, d0; \
	fmov    d9, d0; \
	fmov    d10, d0; \
	fmov    d11, d0

#else // defined(OS_MAC)

#define STACKSIZE 11*16
.macro PROLOGUE
	sub sp, sp, #(11 * 16)
	stp d8, d9, [sp, #(0 * 16)]
	stp d10, d11, [sp, #(1 * 16)]
	stp d12, d13, [sp, #(2 * 16)]
	stp d14, d15, [sp, #(3 * 16)]
	stp x18, x19, [sp, #(4 * 16)]
	stp x20, x21, [sp, #(5 * 16)]
	stp x22, x23, [sp, #(6 * 16)]
	stp x24, x25, [sp, #(7 * 16)]
	stp x26, x27, [sp, #(8 * 16)]
	stp x28, x29, [sp, #(9 * 16)]
	str x30, [sp, #(10 * 16)]
.endm
.macro EPILOGUE
	ldp d8, d9, [sp, #(0 * 16)]
	ldp d10, d11, [sp, #(1 * 16)]
	ldp d12, d13, [sp, #(2 * 16)]
	ldp d14, d15, [sp, #(3 * 16)]
	ldp x18, x19, [sp, #(4 * 16)]
	ldp x20, x21, [sp, #(5 * 16)]
	ldp x22, x23, [sp, #(6 * 16)]
	ldp x24, x25, [sp, #(7 * 16)]
	ldp x26, x27, [sp, #(8 * 16)]
	ldp x28, x29, [sp, #(9 * 16)]
	ldr x30, [sp, #(10 * 16)]
	add sp, sp, #(11 * 16)
.endm
#define GLOB(NAME) \
	.globl _ ## NAME
#define FUN_START(NAME) \
_ ## NAME:
#define FUN_END(NAME)
#define CALL(NAME) \
	bl _ ## NAME
.macro ZERO_ACC
	fmov	d0, xzr
	fmov    d1, d0
	fmov    d2, d0
	fmov    d3, d0
	fmov    d4, d0
	fmov    d5, d0
	fmov    d6, d0
	fmov    d7, d0
	fmov    d8, d0
	fmov    d9, d0
	fmov    d10, d0
	fmov    d11, d0
.endm

#endif





	.text





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- sda
// x11  <- B
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_12x4_lib4)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x9, x10
	add		x13, x12, x10

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x12, #0]
	prfm	PLDL1KEEP, [x13, #0]

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x11, #32]
	prfm	PLDL1KEEP, [x9, #32]
	prfm	PLDL1KEEP, [x12, #32]
	prfm	PLDL1KEEP, [x13, #32]

	// main loop
1:

	ldp		q28, q29, [x11, #(0*16)]
	ldp		q24, q25, [x9, #(0*16)]
	ldp		q20, q21, [x12, #(0*16)]
	ldp		q16, q17, [x13, #(0*16)]

	ldp		q30, q31, [x11, #(2*16)]
	ldp		q26, q27, [x9, #(2*16)]
	ldp		q22, q23, [x12, #(2*16)]
	ldp		q18, q19, [x13, #(2*16)]

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v28.s[1]
	fmla	v2.4s, v24.4s, v28.s[2]
	fmla	v3.4s, v24.4s, v28.s[3]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v28.s[1]
	fmla	v6.4s, v20.4s, v28.s[2]
	fmla	v7.4s, v20.4s, v28.s[3]
	fmla	v8.4s, v16.4s, v28.s[0]
	fmla	v9.4s, v16.4s, v28.s[1]
	fmla	v10.4s, v16.4s, v28.s[2]
	fmla	v11.4s, v16.4s, v28.s[3]


	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
	fmla	v1.4s, v25.4s, v29.s[1]
	prfm	PLDL1KEEP, [x11, #128]
	fmla	v2.4s, v25.4s, v29.s[2]
	fmla	v3.4s, v25.4s, v29.s[3]
	prfm	PLDL1KEEP, [x9, #128]
	fmla	v4.4s, v21.4s, v29.s[0]
	fmla	v5.4s, v21.4s, v29.s[1]
	prfm	PLDL1KEEP, [x12, #128]
	fmla	v6.4s, v21.4s, v29.s[2]
	fmla	v7.4s, v21.4s, v29.s[3]
	prfm	PLDL1KEEP, [x13, #128]
	fmla	v8.4s, v17.4s, v29.s[0]
	fmla	v9.4s, v17.4s, v29.s[1]
	sub		w8, w8, #4
	fmla	v10.4s, v17.4s, v29.s[2]
	fmla	v11.4s, v17.4s, v29.s[3]

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
	fmla	v1.4s, v26.4s, v30.s[1]
	add		x11, x11, #64
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v3.4s, v26.4s, v30.s[3]
	add		x9, x9, #64
	fmla	v4.4s, v22.4s, v30.s[0]
	fmla	v5.4s, v22.4s, v30.s[1]
	add		x12, x12, #64
	fmla	v6.4s, v22.4s, v30.s[2]
	fmla	v7.4s, v22.4s, v30.s[3]
	add		x13, x13, #64
	fmla	v8.4s, v18.4s, v30.s[0]
	fmla	v9.4s, v18.4s, v30.s[1]
	fmla	v10.4s, v18.4s, v30.s[2]
	fmla	v11.4s, v18.4s, v30.s[3]

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
	fmla	v1.4s, v27.4s, v31.s[1]
	fmla	v2.4s, v27.4s, v31.s[2]
	fmla	v3.4s, v27.4s, v31.s[3]
	cmp		w8, #4
	fmla	v4.4s, v23.4s, v31.s[0]
	fmla	v5.4s, v23.4s, v31.s[1]
	fmla	v6.4s, v23.4s, v31.s[2]
	fmla	v7.4s, v23.4s, v31.s[3]
	fmla	v8.4s, v19.4s, v31.s[0]
	fmla	v9.4s, v19.4s, v31.s[1]
	fmla	v10.4s, v19.4s, v31.s[2]
	fmla	v11.4s, v19.4s, v31.s[3]

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	ldp		q28, q29, [x11, #(0*16)]
	ldp		q24, q25, [x9, #(0*16)]
	ldp		q20, q21, [x12, #(0*16)]
	ldp		q16, q17, [x13, #(0*16)]

	ldp		q30, q31, [x11, #(2*16)]
	ldp		q26, q27, [x9, #(2*16)]
	ldp		q22, q23, [x12, #(2*16)]
	ldp		q18, q19, [x13, #(2*16)]

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v28.s[1]
	fmla	v2.4s, v24.4s, v28.s[2]
	fmla	v3.4s, v24.4s, v28.s[3]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v28.s[1]
	fmla	v6.4s, v20.4s, v28.s[2]
	fmla	v7.4s, v20.4s, v28.s[3]
	fmla	v8.4s, v16.4s, v28.s[0]
	fmla	v9.4s, v16.4s, v28.s[1]
	fmla	v10.4s, v16.4s, v28.s[2]
	fmla	v11.4s, v16.4s, v28.s[3]


	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
	fmla	v1.4s, v25.4s, v29.s[1]
	fmla	v2.4s, v25.4s, v29.s[2]
	fmla	v3.4s, v25.4s, v29.s[3]
	prfm	PLDL1KEEP, [x11, #64]
	fmla	v4.4s, v21.4s, v29.s[0]
	fmla	v5.4s, v21.4s, v29.s[1]
	prfm	PLDL1KEEP, [x9, #64]
	fmla	v6.4s, v21.4s, v29.s[2]
	fmla	v7.4s, v21.4s, v29.s[3]
	prfm	PLDL1KEEP, [x12, #64]
	fmla	v8.4s, v17.4s, v29.s[0]
	fmla	v9.4s, v17.4s, v29.s[1]
	sub		w8, w8, #4
	fmla	v10.4s, v17.4s, v29.s[2]
	fmla	v11.4s, v17.4s, v29.s[3]

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
	fmla	v1.4s, v26.4s, v30.s[1]
	add		x11, x11, #64
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v3.4s, v26.4s, v30.s[3]
	add		x9, x9, #64
	fmla	v4.4s, v22.4s, v30.s[0]
	fmla	v5.4s, v22.4s, v30.s[1]
	add		x12, x12, #64
	fmla	v6.4s, v22.4s, v30.s[2]
	fmla	v7.4s, v22.4s, v30.s[3]
	add		x13, x13, #64
	fmla	v8.4s, v18.4s, v30.s[0]
	fmla	v9.4s, v18.4s, v30.s[1]
	fmla	v10.4s, v18.4s, v30.s[2]
	fmla	v11.4s, v18.4s, v30.s[3]

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
	fmla	v1.4s, v27.4s, v31.s[1]
	fmla	v2.4s, v27.4s, v31.s[2]
	fmla	v3.4s, v27.4s, v31.s[3]
	cmp		w8, #4
	fmla	v4.4s, v23.4s, v31.s[0]
	fmla	v5.4s, v23.4s, v31.s[1]
	fmla	v6.4s, v23.4s, v31.s[2]
	fmla	v7.4s, v23.4s, v31.s[3]
	fmla	v8.4s, v19.4s, v31.s[0]
	fmla	v9.4s, v19.4s, v31.s[1]
	fmla	v10.4s, v19.4s, v31.s[2]
	fmla	v11.4s, v19.4s, v31.s[3]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x9, x9, #32
	sub		x12, x12, #32
	sub		x11, x11, #32
	sub		x13, x13, #32

3: // clean1-up loop

	// unroll 0

	ld1		{v28.4s}, [x11], #16
	ld1		{v24.4s}, [x9], #16
	ld1		{v20.4s}, [x12], #16
	ld1		{v16.4s}, [x13], #16
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v28.s[1]
	fmla	v2.4s, v24.4s, v28.s[2]
	fmla	v3.4s, v24.4s, v28.s[3]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v28.s[1]
	fmla	v6.4s, v20.4s, v28.s[2]
	fmla	v7.4s, v20.4s, v28.s[3]
	fmla	v8.4s, v16.4s, v28.s[0]
	fmla	v9.4s, v16.4s, v28.s[1]
	fmla	v10.4s, v16.4s, v28.s[2]
	fmla	v11.4s, v16.4s, v28.s[3]

	sub		w8, w8, #1
	cmp		w8, #0
	bgt		3b

2: // return



#else // cortex a53



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x9, x10
	add		x13, x12, x10

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x12, #0]
	prfm	PLDL1KEEP, [x13, #0]

	// preload
	ld1		{v24.4s, v25.4s}, [x9], #32
	ld1		{v28.4s, v29.4s}, [x11], #32
	ld1		{v20.4s, v21.4s}, [x12], #32
	ld1		{v16.4s, v17.4s}, [x13], #32

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x11, #32]
	prfm	PLDL1KEEP, [x9, #32]
	prfm	PLDL1KEEP, [x12, #32]
	prfm	PLDL1KEEP, [x13, #32]

	// main loop
1:

	// unroll 0
	ld1		{v26.4s}, [x9], #16
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v28.s[1]
	ld1		{v27.4s}, [x9], #16
	fmla	v2.4s, v24.4s, v28.s[2]
	fmla	v3.4s, v24.4s, v28.s[3]
	ld1		{v30.4s}, [x11], #16
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v28.s[1]
	ld1		{v31.4s}, [x11], #16
	fmla	v6.4s, v20.4s, v28.s[2]
	fmla	v7.4s, v20.4s, v28.s[3]
	ld1		{v22.4s}, [x12], #16
	fmla	v8.4s, v16.4s, v28.s[0]
	fmla	v9.4s, v16.4s, v28.s[1]
	ld1		{v23.4s}, [x12], #16
	fmla	v10.4s, v16.4s, v28.s[2]
	fmla	v11.4s, v16.4s, v28.s[3]

	// unroll 1
	ld1		{v18.4s}, [x13], #16
	fmla	v0.4s, v25.4s, v29.s[0]
	fmla	v1.4s, v25.4s, v29.s[1]
	ld1		{v19.4s}, [x13], #16
	fmla	v2.4s, v25.4s, v29.s[2]
	fmla	v3.4s, v25.4s, v29.s[3]
	prfm	PLDL1KEEP, [x11, #64]
	fmla	v4.4s, v21.4s, v29.s[0]
	fmla	v5.4s, v21.4s, v29.s[1]
	prfm	PLDL1KEEP, [x9, #64]
	fmla	v6.4s, v21.4s, v29.s[2]
	fmla	v7.4s, v21.4s, v29.s[3]
	prfm	PLDL1KEEP, [x12, #64]
	fmla	v8.4s, v17.4s, v29.s[0]
	fmla	v9.4s, v17.4s, v29.s[1]
	prfm	PLDL1KEEP, [x13, #128]
	fmla	v10.4s, v17.4s, v29.s[2]
	fmla	v11.4s, v17.4s, v29.s[3]

	// unroll 2
	ld1		{v24.4s}, [x9], #16
	fmla	v0.4s, v26.4s, v30.s[0]
	fmla	v1.4s, v26.4s, v30.s[1]
	ld1		{v25.4s}, [x9], #16
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v3.4s, v26.4s, v30.s[3]
	ld1		{v28.4s}, [x11], #16
	fmla	v4.4s, v22.4s, v30.s[0]
	fmla	v5.4s, v22.4s, v30.s[1]
	ld1		{v29.4s}, [x11], #16
	fmla	v6.4s, v22.4s, v30.s[2]
	fmla	v7.4s, v22.4s, v30.s[3]
	ld1		{v20.4s}, [x12], #16
	fmla	v8.4s, v18.4s, v30.s[0]
	fmla	v9.4s, v18.4s, v30.s[1]
	ld1		{v21.4s}, [x12], #16
	fmla	v10.4s, v18.4s, v30.s[2]
	fmla	v11.4s, v18.4s, v30.s[3]

	// unroll 3
	ld1		{v16.4s}, [x13], #16
	fmla	v0.4s, v27.4s, v31.s[0]
	fmla	v1.4s, v27.4s, v31.s[1]
	ld1		{v17.4s}, [x13], #16
	fmla	v2.4s, v27.4s, v31.s[2]
	fmla	v3.4s, v27.4s, v31.s[3]
	sub		w8, w8, #4
	fmla	v4.4s, v23.4s, v31.s[0]
	fmla	v5.4s, v23.4s, v31.s[1]
	cmp		w8, #4
	fmla	v6.4s, v23.4s, v31.s[2]
	fmla	v7.4s, v23.4s, v31.s[3]
	fmla	v8.4s, v19.4s, v31.s[0]
	fmla	v9.4s, v19.4s, v31.s[1]
	fmla	v10.4s, v19.4s, v31.s[2]
	fmla	v11.4s, v19.4s, v31.s[3]

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ld1		{v26.4s}, [x9], #16
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v28.s[1]
	ld1		{v27.4s}, [x9], #16
	fmla	v2.4s, v24.4s, v28.s[2]
	fmla	v3.4s, v24.4s, v28.s[3]
	ld1		{v30.4s}, [x11], #16
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v28.s[1]
	ld1		{v31.4s}, [x11], #16
	fmla	v6.4s, v20.4s, v28.s[2]
	fmla	v7.4s, v20.4s, v28.s[3]
	ld1		{v22.4s}, [x12], #16
	fmla	v8.4s, v16.4s, v28.s[0]
	fmla	v9.4s, v16.4s, v28.s[1]
	ld1		{v23.4s}, [x12], #16
	fmla	v10.4s, v16.4s, v28.s[2]
	fmla	v11.4s, v16.4s, v28.s[3]

	// unroll 1
	ld1		{v18.4s}, [x13], #16
	fmla	v0.4s, v25.4s, v29.s[0]
	fmla	v1.4s, v25.4s, v29.s[1]
	ld1		{v19.4s}, [x13], #16
	fmla	v2.4s, v25.4s, v29.s[2]
	fmla	v3.4s, v25.4s, v29.s[3]
//	prfm	PLDL1KEEP, [x11, #64]
	fmla	v4.4s, v21.4s, v29.s[0]
	fmla	v5.4s, v21.4s, v29.s[1]
//	prfm	PLDL1KEEP, [x9, #64]
	fmla	v6.4s, v21.4s, v29.s[2]
	fmla	v7.4s, v21.4s, v29.s[3]
//	prfm	PLDL1KEEP, [x12, #64]
	fmla	v8.4s, v17.4s, v29.s[0]
	fmla	v9.4s, v17.4s, v29.s[1]
	sub		w8, w8, #4
	fmla	v10.4s, v17.4s, v29.s[2]
	fmla	v11.4s, v17.4s, v29.s[3]

	// unroll 2
//	ld1		{v24.4s}, [x9], #16
	fmla	v0.4s, v26.4s, v30.s[0]
	fmla	v1.4s, v26.4s, v30.s[1]
//	ld1		{v25.4s}, [x9], #16
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v3.4s, v26.4s, v30.s[3]
//	ld1		{v28.4s}, [x11], #16
	fmla	v4.4s, v22.4s, v30.s[0]
	fmla	v5.4s, v22.4s, v30.s[1]
//	ld1		{v29.4s}, [x11], #16
	fmla	v6.4s, v22.4s, v30.s[2]
	fmla	v7.4s, v22.4s, v30.s[3]
//	ld1		{v20.4s}, [x12], #16
	fmla	v8.4s, v18.4s, v30.s[0]
	fmla	v9.4s, v18.4s, v30.s[1]
//	ld1		{v21.4s}, [x12], #16
	fmla	v10.4s, v18.4s, v30.s[2]
	fmla	v11.4s, v18.4s, v30.s[3]

	// unroll 3
//	ld1		{v16.4s}, [x13], #16
	fmla	v0.4s, v27.4s, v31.s[0]
	fmla	v1.4s, v27.4s, v31.s[1]
//	ld1		{v17.4s}, [x13], #16
	fmla	v2.4s, v27.4s, v31.s[2]
	fmla	v3.4s, v27.4s, v31.s[3]
	cmp		w8, #4
	fmla	v4.4s, v23.4s, v31.s[0]
	fmla	v5.4s, v23.4s, v31.s[1]
	fmla	v6.4s, v23.4s, v31.s[2]
	fmla	v7.4s, v23.4s, v31.s[3]
	fmla	v8.4s, v19.4s, v31.s[0]
	fmla	v9.4s, v19.4s, v31.s[1]
	fmla	v10.4s, v19.4s, v31.s[2]
	fmla	v11.4s, v19.4s, v31.s[3]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x9, x9, #32
	sub		x12, x12, #32
	sub		x11, x11, #32
	sub		x13, x13, #32

3: // clean1-up loop

	// unroll 0

	ld1		{v28.4s}, [x11], #16
	ld1		{v24.4s}, [x9], #16
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v28.s[1]
	fmla	v2.4s, v24.4s, v28.s[2]
	fmla	v3.4s, v24.4s, v28.s[3]
	ld1		{v20.4s}, [x12], #16
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v28.s[1]
	fmla	v6.4s, v20.4s, v28.s[2]
	fmla	v7.4s, v20.4s, v28.s[3]
	ld1		{v16.4s}, [x13], #16
	fmla	v8.4s, v16.4s, v28.s[0]
	fmla	v9.4s, v16.4s, v28.s[1]
	fmla	v10.4s, v16.4s, v28.s[2]
	fmla	v11.4s, v16.4s, v28.s[3]

	sub		w8, w8, #1
	cmp		w8, #0
	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_12x4_lib4)
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
// x9   <- inv_diag_E
//
// output arguments:
// x8   <- E
// x9   <- inv_diag_E

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRSM_RLT_INV_12X4_LIB4
#else
	.align 4
	FUN_START(inner_edge_trsm_rlt_inv_12x4_lib4)
#endif
	
	// first column
	ldr			s16, [x9, #0] // E_inv[0]
	fmul		v0.4s, v0.4s, v16.s[0]
	fmul		v4.4s, v4.4s, v16.s[0]
	fmul		v8.4s, v8.4s, v16.s[0]

	// second column
	ldr			s16, [x8, #4] // E[1+4*0]
	fmls		v1.4s, v0.4s, v16.s[0]
	fmls		v5.4s, v4.4s, v16.s[0]
	fmls		v9.4s, v8.4s, v16.s[0]
	ldr			s16, [x9, #4] // E_inv[1]
	fmul		v1.4s, v1.4s, v16.s[0]
	fmul		v5.4s, v5.4s, v16.s[0]
	fmul		v9.4s, v9.4s, v16.s[0]

	// third column
	ldr			s16, [x8, #8] // E[2+4*0]
	fmls		v2.4s, v0.4s, v16.s[0]
	fmls		v6.4s, v4.4s, v16.s[0]
	fmls		v10.4s, v8.4s, v16.s[0]
	ldr			s16, [x8, #24] // E[2+4*1]
	fmls		v2.4s, v1.4s, v16.s[0]
	fmls		v6.4s, v5.4s, v16.s[0]
	fmls		v10.4s, v9.4s, v16.s[0]
	ldr			s16, [x9, #8] // E_inv[2]
	fmul		v2.4s, v2.4s, v16.s[0]
	fmul		v6.4s, v6.4s, v16.s[0]
	fmul		v10.4s, v10.4s, v16.s[0]

	// forth column
	ldr			s16, [x8, #12] // E[3+4*0]
	fmls		v3.4s, v0.4s, v16.s[0]
	fmls		v7.4s, v4.4s, v16.s[0]
	fmls		v11.4s, v8.4s, v16.s[0]
	ldr			s16, [x8, #28] // E[3+4*1]
	fmls		v3.4s, v1.4s, v16.s[0]
	fmls		v7.4s, v5.4s, v16.s[0]
	fmls		v11.4s, v9.4s, v16.s[0]
	ldr			s16, [x8, #44] // E[3+4*1]
	fmls		v3.4s, v2.4s, v16.s[0]
	fmls		v7.4s, v6.4s, v16.s[0]
	fmls		v11.4s, v10.4s, v16.s[0]
	ldr			s16, [x9, #12] // E_inv[2]
	fmul		v3.4s, v3.4s, v16.s[0]
	fmul		v7.4s, v7.4s, v16.s[0]
	fmul		v11.4s, v11.4s, v16.s[0]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	FUN_END(inner_edge_trsm_rlt_inv_12x4_lib4)
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
// x8   <- E
// x9   <- inv_diag_E
// w10  <- n1
//
// output arguments:
// x8   <- E
// x9   <- inv_diag_E
// w10  <- n1

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRSM_RLT_INV_12X4_VS_LIB4
#else
	.align 4
	FUN_START(inner_edge_trsm_rlt_inv_12x4_vs_lib4)
#endif
	
	// first column
	ldr			s16, [x9, #0] // E_inv[0]
	fmul		v0.4s, v0.4s, v16.s[0]
	fmul		v4.4s, v4.4s, v16.s[0]
	fmul		v8.4s, v8.4s, v16.s[0]
	cmp			w10, #2
	blt			0f // return

	// second column
	ldr			s16, [x8, #4] // E[1+4*0]
	fmls		v1.4s, v0.4s, v16.s[0]
	fmls		v5.4s, v4.4s, v16.s[0]
	fmls		v9.4s, v8.4s, v16.s[0]
	ldr			s16, [x9, #4] // E_inv[1]
	fmul		v1.4s, v1.4s, v16.s[0]
	fmul		v5.4s, v5.4s, v16.s[0]
	fmul		v9.4s, v9.4s, v16.s[0]
	cmp			w10, #3
	blt			0f // return

	// third column
	ldr			s16, [x8, #8] // E[2+4*0]
	fmls		v2.4s, v0.4s, v16.s[0]
	fmls		v6.4s, v4.4s, v16.s[0]
	fmls		v10.4s, v8.4s, v16.s[0]
	ldr			s16, [x8, #24] // E[2+4*1]
	fmls		v2.4s, v1.4s, v16.s[0]
	fmls		v6.4s, v5.4s, v16.s[0]
	fmls		v10.4s, v9.4s, v16.s[0]
	ldr			s16, [x9, #8] // E_inv[2]
	fmul		v2.4s, v2.4s, v16.s[0]
	fmul		v6.4s, v6.4s, v16.s[0]
	fmul		v10.4s, v10.4s, v16.s[0]
	cmp			w10, #4
	blt			0f // return

	// forth column
	ldr			s16, [x8, #12] // E[3+4*0]
	fmls		v3.4s, v0.4s, v16.s[0]
	fmls		v7.4s, v4.4s, v16.s[0]
	fmls		v11.4s, v8.4s, v16.s[0]
	ldr			s16, [x8, #28] // E[3+4*1]
	fmls		v3.4s, v1.4s, v16.s[0]
	fmls		v7.4s, v5.4s, v16.s[0]
	fmls		v11.4s, v9.4s, v16.s[0]
	ldr			s16, [x8, #44] // E[3+4*1]
	fmls		v3.4s, v2.4s, v16.s[0]
	fmls		v7.4s, v6.4s, v16.s[0]
	fmls		v11.4s, v10.4s, v16.s[0]
	ldr			s16, [x9, #12] // E_inv[2]
	fmul		v3.4s, v3.4s, v16.s[0]
	fmul		v7.4s, v7.4s, v16.s[0]
	fmul		v11.4s, v11.4s, v16.s[0]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	FUN_END(inner_edge_trsm_rlt_inv_12x4_vs_lib4)
#endif
#endif





// subroutine
//
// cholesky factorization 
//
// input arguments:
// x8   <- inv_diag_D
//
// output arguments:
// x8   <- inv_diag_D

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_POTRF_12X4_LIB4
#else
	.p2align 4
	FUN_START(inner_edge_potrf_12x4_lib4)
#endif
	
	fmov		s16, 1.0e+0 // 1.0

	// first column
	ins			v17.s[0], v0.s[0]
	fcmpe		s17, #0.0
	ble			1f
	fsqrt		s17, s17
	fdiv		s18, s16, s17
2:
	str			s18, [x8, #0]
	fmul		v0.4s, v0.4s, v18.s[0]
	fmul		v4.4s, v4.4s, v18.s[0]
	fmul		v8.4s, v8.4s, v18.s[0]

	// second column
	fmls		v1.4s, v0.4s, v0.s[1]
	fmls		v5.4s, v4.4s, v0.s[1]
	fmls		v9.4s, v8.4s, v0.s[1]
	ins			v17.s[0], v1.s[1]
	fcmpe		s17, #0.0
	ble			3f
	fsqrt		s17, s17
	fdiv		s18, s16, s17
4:
	str			s18, [x8, #4]
	fmul		v1.4s, v1.4s, v18.s[0]
	fmul		v5.4s, v5.4s, v18.s[0]
	fmul		v9.4s, v9.4s, v18.s[0]

	// third column
	fmls		v2.4s, v0.4s, v0.s[2]
	fmls		v6.4s, v4.4s, v0.s[2]
	fmls		v10.4s, v8.4s, v0.s[2]
	fmls		v2.4s, v1.4s, v1.s[2]
	fmls		v6.4s, v5.4s, v1.s[2]
	fmls		v10.4s, v9.4s, v1.s[2]
	ins			v17.s[0], v2.s[2]
	fcmpe		s17, #0.0
	ble			5f
	fsqrt		s17, s17
	fdiv		s18, s16, s17
6:
	str			s18, [x8, #8]
	fmul		v2.4s, v2.4s, v18.s[0]
	fmul		v6.4s, v6.4s, v18.s[0]
	fmul		v10.4s, v10.4s, v18.s[0]

	// fourth column
	fmls		v3.4s, v0.4s, v0.s[3]
	fmls		v7.4s, v4.4s, v0.s[3]
	fmls		v11.4s, v8.4s, v0.s[3]
	fmls		v3.4s, v1.4s, v1.s[3]
	fmls		v7.4s, v5.4s, v1.s[3]
	fmls		v11.4s, v9.4s, v1.s[3]
	fmls		v3.4s, v2.4s, v2.s[3]
	fmls		v7.4s, v6.4s, v2.s[3]
	fmls		v11.4s, v10.4s, v2.s[3]
	ins			v17.s[0], v3.s[3]
	fcmpe		s17, #0.0
	ble			7f
	fsqrt		s17, s17
	fdiv		s18, s16, s17
8:
	str			s18, [x8, #12]
	fmul		v3.4s, v3.4s, v18.s[0]
	fmul		v7.4s, v7.4s, v18.s[0]
	fmul		v11.4s, v11.4s, v18.s[0]

	b			0f

1:
	fmov		d18, xzr
	b			2b

3:
	fmov		d18, xzr
	b			4b

5:
	fmov		d18, xzr
	b			6b

7:
	fmov		d18, xzr

0:
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_potrf_12x4_lib4)
#endif





// subroutine
//
// cholesky factorization 
//
// input arguments:
// x8   <- inv_diag_D
// x9   <- n1
//
// output arguments:
// x8   <- inv_diag_D
// x9   <- n1

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_POTRF_12X4_VS_LIB4
#else
	.p2align 4
	FUN_START(inner_edge_potrf_12x4_vs_lib4)
#endif
	
	fmov		s16, 1.0e+0 // 1.0

	// first column
	ins			v17.s[0], v0.s[0]
	fcmpe		s17, #0.0
	ble			1f
	fsqrt		s17, s17
	fdiv		s18, s16, s17
2:
	str			s18, [x8, #0]
	fmul		v0.4s, v0.4s, v18.s[0]
	fmul		v4.4s, v4.4s, v18.s[0]
	fmul		v8.4s, v8.4s, v18.s[0]
	cmp		w9, #2
	blt		0f // return

	// second column
	fmls		v1.4s, v0.4s, v0.s[1]
	fmls		v5.4s, v4.4s, v0.s[1]
	fmls		v9.4s, v8.4s, v0.s[1]
	ins			v17.s[0], v1.s[1]
	fcmpe		s17, #0.0
	ble			3f
	fsqrt		s17, s17
	fdiv		s18, s16, s17
4:
	str			s18, [x8, #4]
	fmul		v1.4s, v1.4s, v18.s[0]
	fmul		v5.4s, v5.4s, v18.s[0]
	fmul		v9.4s, v9.4s, v18.s[0]
	cmp		w9, #3
	blt		0f // return

	// third column
	fmls		v2.4s, v0.4s, v0.s[2]
	fmls		v6.4s, v4.4s, v0.s[2]
	fmls		v10.4s, v8.4s, v0.s[2]
	fmls		v2.4s, v1.4s, v1.s[2]
	fmls		v6.4s, v5.4s, v1.s[2]
	fmls		v10.4s, v9.4s, v1.s[2]
	ins			v17.s[0], v2.s[2]
	fcmpe		s17, #0.0
	ble			5f
	fsqrt		s17, s17
	fdiv		s18, s16, s17
6:
	str			s18, [x8, #8]
	fmul		v2.4s, v2.4s, v18.s[0]
	fmul		v6.4s, v6.4s, v18.s[0]
	fmul		v10.4s, v10.4s, v18.s[0]
	cmp		w9, #4
	blt		0f // return

	// fourth column
	fmls		v3.4s, v0.4s, v0.s[3]
	fmls		v7.4s, v4.4s, v0.s[3]
	fmls		v11.4s, v8.4s, v0.s[3]
	fmls		v3.4s, v1.4s, v1.s[3]
	fmls		v7.4s, v5.4s, v1.s[3]
	fmls		v11.4s, v9.4s, v1.s[3]
	fmls		v3.4s, v2.4s, v2.s[3]
	fmls		v7.4s, v6.4s, v2.s[3]
	fmls		v11.4s, v10.4s, v2.s[3]
	ins			v17.s[0], v3.s[3]
	fcmpe		s17, #0.0
	ble			7f
	fsqrt		s17, s17
	fdiv		s18, s16, s17
8:
	str			s18, [x8, #12]
	fmul		v3.4s, v3.4s, v18.s[0]
	fmul		v7.4s, v7.4s, v18.s[0]
	fmul		v11.4s, v11.4s, v18.s[0]

	b			0f

1:
	fmov		d18, xzr
	b			2b

3:
	fmov		d18, xzr
	b			4b

5:
	fmov		d18, xzr
	b			6b

7:
	fmov		d18, xzr

0:
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_potrf_12x4_vs_lib4)
#endif





// subroutine
//
// input arguments:
// x8   <- alpha
// x9   <- beta
// x10  <- C
// x11  <- sdc
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_12X4_LIB4
#else
	.align	4
	FUN_START(inner_scale_ab_12x4_lib4)
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

	fcmpe	s29, #0.0
	beq		0f

	add		x12, x10, x11
	add		x13, x12, x11

	ld1		{v24.4s, v25.4s, v26.4s, v27.4s}, [x10], #64
	fmla	v0.4s, v24.4s, v29.s[0]
	fmla	v1.4s, v25.4s, v29.s[0]
	fmla	v2.4s, v26.4s, v29.s[0]
	fmla	v3.4s, v27.4s, v29.s[0]

	ld1		{v24.4s, v25.4s, v26.4s, v27.4s}, [x12], #64
	fmla	v4.4s, v24.4s, v29.s[0]
	fmla	v5.4s, v25.4s, v29.s[0]
	fmla	v6.4s, v26.4s, v29.s[0]
	fmla	v7.4s, v27.4s, v29.s[0]

	ld1		{v24.4s, v25.4s, v26.4s, v27.4s}, [x13], #64
	fmla	v8.4s, v24.4s, v29.s[0]
	fmla	v9.4s, v25.4s, v29.s[0]
	fmla	v10.4s, v26.4s, v29.s[0]
	fmla	v11.4s, v27.4s, v29.s[0]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_12x4_lib4)
#endif





// subroutine
//
// input arguments:
// x8   <- beta
// x9  <- C
// x10  <- sdc
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M1B_12X4_LIB4
#else
	.align	4
	FUN_START(inner_scale_m1b_12x4_lib4)
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
	fneg	v8.4s, v8.4s
	fneg	v9.4s, v9.4s
	fneg	v10.4s, v10.4s
	fneg	v11.4s, v11.4s

	fcmpe	s29, #0.0
	beq		0f

	add		x12, x9, x10
	add		x13, x12, x10

	ld1		{v24.4s, v25.4s, v26.4s, v27.4s}, [x9], #64
	fmla	v0.4s, v24.4s, v29.s[0]
	fmla	v1.4s, v25.4s, v29.s[0]
	fmla	v2.4s, v26.4s, v29.s[0]
	fmla	v3.4s, v27.4s, v29.s[0]

	ld1		{v24.4s, v25.4s, v26.4s, v27.4s}, [x12], #64
	fmla	v4.4s, v24.4s, v29.s[0]
	fmla	v5.4s, v25.4s, v29.s[0]
	fmla	v6.4s, v26.4s, v29.s[0]
	fmla	v7.4s, v27.4s, v29.s[0]

	ld1		{v24.4s, v25.4s, v26.4s, v27.4s}, [x13], #64
	fmla	v8.4s, v24.4s, v29.s[0]
	fmla	v9.4s, v25.4s, v29.s[0]
	fmla	v10.4s, v26.4s, v29.s[0]
	fmla	v11.4s, v27.4s, v29.s[0]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m1b_12x4_lib4)
#endif





// subroutine
//
// input arguments:
// x8  <- C
// x9  <- sdc
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_11_12X4_LIB4
#else
	.align	4
	FUN_START(inner_scale_11_12x4_lib4)
#endif

	add		x12, x8, x9
	add		x13, x12, x9

	ld1		{v24.4s, v25.4s, v26.4s, v27.4s}, [x8], #64
	fadd	v0.4s, v24.4s, v0.4s
	fadd	v1.4s, v25.4s, v1.4s
	fadd	v2.4s, v26.4s, v2.4s
	fadd	v3.4s, v27.4s, v3.4s

	ld1		{v24.4s, v25.4s, v26.4s, v27.4s}, [x12], #64
	fadd	v4.4s, v24.4s, v4.4s
	fadd	v5.4s, v25.4s, v5.4s
	fadd	v6.4s, v26.4s, v6.4s
	fadd	v7.4s, v27.4s, v7.4s

	ld1		{v24.4s, v25.4s, v26.4s, v27.4s}, [x13], #64
	fadd	v8.4s, v24.4s, v8.4s
	fadd	v9.4s, v25.4s, v9.4s
	fadd	v10.4s, v26.4s, v10.4s
	fadd	v11.4s, v27.4s, v11.4s

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_11_12x4_lib4)
#endif





// subroutine
//
// input arguments:
// x8  <- C
// x9  <- sdc
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M11_12X4_LIB4
#else
	.align	4
	FUN_START(inner_scale_m11_12x4_lib4)
#endif

	add		x12, x8, x9
	add		x13, x12, x9

	ld1		{v24.4s, v25.4s, v26.4s, v27.4s}, [x8], #64
	fsub	v0.4s, v24.4s, v0.4s
	fsub	v1.4s, v25.4s, v1.4s
	fsub	v2.4s, v26.4s, v2.4s
	fsub	v3.4s, v27.4s, v3.4s

	ld1		{v24.4s, v25.4s, v26.4s, v27.4s}, [x12], #64
	fsub	v4.4s, v24.4s, v4.4s
	fsub	v5.4s, v25.4s, v5.4s
	fsub	v6.4s, v26.4s, v6.4s
	fsub	v7.4s, v27.4s, v7.4s

	ld1		{v24.4s, v25.4s, v26.4s, v27.4s}, [x13], #64
	fsub	v8.4s, v24.4s, v8.4s
	fsub	v9.4s, v25.4s, v9.4s
	fsub	v10.4s, v26.4s, v10.4s
	fsub	v11.4s, v27.4s, v11.4s

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m11_12x4_lib4)
#endif





// subroutine
//
// input arguments:
// x8   <- D
// x9   <- sdd
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_12X4_LIB4
#else
	.align 4
	FUN_START(inner_store_12x4_lib4)
#endif

	add		x10, x8, x9
	add		x11, x10, x9

	st1		{v0.4s, v1.4s, v2.4s, v3.4s}, [x8], #64
	st1		{v4.4s, v5.4s, v6.4s, v7.4s}, [x10], #64
	st1		{v8.4s, v9.4s, v10.4s, v11.4s}, [x11], #64

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_12x4_lib4)
#endif





// subroutine
//
// input arguments:
// x8   <- D
// x9   <- sdd
// x10  <- km
// x11  <- kn
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_12X4_VS_LIB4
#else
	.align 4
	FUN_START(inner_store_12x4_vs_lib4)
#endif

	add		x12, x8, x9
	add		x13, x12, x9

	cmp		w10, #12
	bge		1f

	ldp		q24, q25, [x13, #(0*16)]
	ldp		q26, q27, [x13, #(2*16)]

	// 4th row
	ins		v8.s[3], v24.s[3]
	ins		v9.s[3], v25.s[3]
	ins		v10.s[3], v26.s[3]
	ins		v11.s[3], v27.s[3]
	cmp		w10, #11
	bge		1f
	// 3th row
	ins		v8.s[2], v24.s[2]
	ins		v9.s[2], v25.s[2]
	ins		v10.s[2], v26.s[2]
	ins		v11.s[2], v27.s[2]
	cmp		w10, #10
	bge		1f
	// 2nd row
	ins		v8.s[1], v24.s[1]
	ins		v9.s[1], v25.s[1]
	ins		v10.s[1], v26.s[1]
	ins		v11.s[1], v27.s[1]
	cmp		w10, #9
	bge		1f
	// 1st row
	ins		v8.s[0], v24.s[0]
	ins		v9.s[0], v25.s[0]
	ins		v10.s[0], v26.s[0]
	ins		v11.s[0], v27.s[0]

1:
	// 1st col
	str		q0, [x8, #(0*16)]
	str		q4, [x12, #(0*16)]
	str		q8, [x13, #(0*16)]
	cmp		w11, #2
	blt		0f
	// 2nd col
	str		q1, [x8, #(1*16)]
	str		q5, [x12, #(1*16)]
	str		q9, [x13, #(1*16)]
	cmp		w11, #3
	blt		0f
	// 3rd col
	str		q2, [x8, #(2*16)]
	str		q6, [x12, #(2*16)]
	str		q10, [x13, #(2*16)]
	beq		0f
	// 4th col
	str		q3, [x8, #(3*16)]
	str		q7, [x12, #(3*16)]
	str		q11, [x13, #(3*16)]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_12x4_vs_lib4)
#endif





// subroutine
//
// input arguments:
// x8   <- D
// x9   <- sdd
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_12X4_LIB4
#else
	.align 4
	FUN_START(inner_store_l_12x4_lib4)
#endif

	ldr		q16, [x8, #16]
	ldr		q17, [x8, #32]
	ldr		q18, [x8, #48]

	ins		v1.s[0], v16.s[0]
	ins		v2.d[0], v17.d[0]
	ins		v3.d[0], v18.d[0]
	ins		v3.s[2], v18.s[2]

	add		x10, x8, x9
	add		x11, x10, x9

	st1		{v0.4s, v1.4s, v2.4s, v3.4s}, [x8], #64
	st1		{v4.4s, v5.4s, v6.4s, v7.4s}, [x10], #64
	st1		{v8.4s, v9.4s, v10.4s, v11.4s}, [x11], #64

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_12x4_lib4)
#endif





// subroutine
//
// input arguments:
// x8   <- D
// x9   <- sdd
// x10  <- km
// x11  <- kn
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_12X4_VS_LIB4
#else
	.align 4
	FUN_START(inner_store_l_12x4_vs_lib4)
#endif

	add		x12, x8, x9
	add		x13, x12, x9

	cmp		w10, #12
	bge		1f

	ldp		q24, q25, [x13, #(0*16)]
	ldp		q26, q27, [x13, #(2*16)]

	// 4th row
	ins		v8.s[3], v24.s[3]
	ins		v9.s[3], v25.s[3]
	ins		v10.s[3], v26.s[3]
	ins		v11.s[3], v27.s[3]
	cmp		w10, #11
	bge		1f
	// 3th row
	ins		v8.s[2], v24.s[2]
	ins		v9.s[2], v25.s[2]
	ins		v10.s[2], v26.s[2]
	ins		v11.s[2], v27.s[2]
	cmp		w10, #10
	bge		1f
	// 2nd row
	ins		v8.s[1], v24.s[1]
	ins		v9.s[1], v25.s[1]
	ins		v10.s[1], v26.s[1]
	ins		v11.s[1], v27.s[1]
	cmp		w10, #9
	bge		1f
	// 1st row
	ins		v8.s[0], v24.s[0]
	ins		v9.s[0], v25.s[0]
	ins		v10.s[0], v26.s[0]
	ins		v11.s[0], v27.s[0]

1:
	ldr		q16, [x8, #16]
	ldr		q17, [x8, #32]
	ldr		q18, [x8, #48]

	ins		v1.s[0], v16.s[0]
	ins		v2.d[0], v17.d[0]
	ins		v3.d[0], v18.d[0]
	ins		v3.s[2], v18.s[2]

	// 1st col
	str		q0, [x8, #(0*16)]
	str		q4, [x12, #(0*16)]
	str		q8, [x13, #(0*16)]
	cmp		w11, #2
	blt		0f
	// 2nd col
	str		q1, [x8, #(1*16)]
	str		q5, [x12, #(1*16)]
	str		q9, [x13, #(1*16)]
	cmp		w11, #3
	blt		0f
	// 3rd col
	str		q2, [x8, #(2*16)]
	str		q6, [x12, #(2*16)]
	str		q10, [x13, #(2*16)]
	beq		0f
	// 4th col
	str		q3, [x8, #(3*16)]
	str		q7, [x12, #(3*16)]
	str		q11, [x13, #(3*16)]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_12x4_vs_lib4)
#endif





//                                w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8
// void kernel_sgemm_nt_12x4_lib4(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd)

	.align	4
	GLOB(kernel_sgemm_nt_12x4_lib4)
	FUN_START(kernel_sgemm_nt_12x4_lib4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #4 // 16*sda
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
	mov		w11, w7 // C
	lsl		w11, w11, #4 // 16*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // sdd
	lsl		w9, w9, #4 // 16*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_nt_12x4_lib4)





//                               w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8     sp+16   sp+24
// void kernel_sgemm_nt_12x4_lib4(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, int m1, int n1)

	.align	4
	GLOB(kernel_sgemm_nt_12x4_vs_lib4)
	FUN_START(kernel_sgemm_nt_12x4_vs_lib4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #4 // 16*sda
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
	mov		w11, w7 // C
	lsl		w11, w11, #4 // 16*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // sdd
	lsl		w9, w9, #4 // 16*sdd
	ldr		w10, [sp, #(STACKSIZE + 16)] // m1
	ldr		w11, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_nt_12x4_vs_lib4)





//                                 w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8
// void kernel_ssyrk_nt_l_12x4_lib4(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd)

	.align	4
	GLOB(kernel_ssyrk_nt_l_12x4_lib4)
	FUN_START(kernel_ssyrk_nt_l_12x4_lib4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #4 // 16*sda
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
	mov		w11, w7 // C
	lsl		w11, w11, #4 // 16*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // sdd
	lsl		w9, w9, #4 // 16*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_L_12X4_LIB4
#else
	CALL(inner_store_l_12x4_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_ssyrk_nt_l_12x4_lib4)





//                                    w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8     sp+16   sp+24
// void kernel_ssyrk_nt_l_12x4_vs_lib4(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, int m1, int n1)

	.align	4
	GLOB(kernel_ssyrk_nt_l_12x4_vs_lib4)
	FUN_START(kernel_ssyrk_nt_l_12x4_vs_lib4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #4 // 16*sda
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
	mov		w11, w7 // C
	lsl		w11, w11, #4 // 16*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // sdd
	lsl		w9, w9, #4 // 16*sdd
	ldr		w10, [sp, #(STACKSIZE + 16)] // m1
	ldr		w11, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_12X4_VS_LIB4
#else
	CALL(inner_store_l_12x4_vs_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_ssyrk_nt_l_12x4_vs_lib4)





//                                      w0        x1         w2        x3        x4            x5         w6       x7         sp+0     sp+8       sp+16
// void kernel_strsm_nt_rl_inv_12x4_lib4(int kmax, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, double *E, double *inv_diag_E);

	.align	4
	GLOB(kernel_strsm_nt_rl_inv_12x4_lib4)
	FUN_START(kernel_strsm_nt_rl_inv_12x4_lib4)



	PROLOGUE



	ZERO_ACC



	// call inner kernel dgemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #4 // 1326*sda
	mov		x11, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_12x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x4 // beta
	mov		x9, x5 // C
	mov		w10, w6 // sdc
	lsl		w10, w10, #4 // 16*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 8)] // E
	ldr		x9, [sp, #(STACKSIZE + 16)] // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_12X4_LIB4
#else
	CALL(inner_edge_trsm_rlt_inv_12x4_lib4)
#endif



	// store
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // sdc
	lsl		w9, w9, #4 // 16*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_strsm_nt_rl_inv_12x4_lib4)





//                                         w0        x1         w2        x3        x4            x5         w6       x7         sp+0     sp+8       sp+16               sp+24   sp+32
// void kernel_strsm_nt_rl_inv_12x4_vs_lib4(int kmax, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, double *E, double *inv_diag_E, int m1, int n1);

	.align	4
	GLOB(kernel_strsm_nt_rl_inv_12x4_vs_lib4)
	FUN_START(kernel_strsm_nt_rl_inv_12x4_vs_lib4)



	PROLOGUE



	ZERO_ACC



	// call inner kernel dgemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #4 // 16*sda
	mov		x11, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_12x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x4 // beta
	mov		x9, x5 // C
	mov		w10, w6 // sdc
	lsl		w10, w10, #4 // 16*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
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



	// store
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // sdd
	lsl		w9, w9, #4 // 16*sdd
	ldr		w10, [sp, #(STACKSIZE + 24)] // m1
	ldr		w11, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_strsm_nt_rl_inv_12x4_vs_lib4)





//                                  w0        x1         w2        x3        x4         w5       x6         w7       sp+0
// void kernel_spotrf_nt_l_12x4_lib4(int kmax, double *A, int sda, double *B, double *C, int sdc, double *D, int sdd, double *inv_diag_D);

	.align	4
	GLOB(kernel_spotrf_nt_l_12x4_lib4)
	FUN_START(kernel_spotrf_nt_l_12x4_lib4)



	PROLOGUE



	ZERO_ACC



	// call inner kernel dsyrk l nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #4 // 16*sda
	mov		x11, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_12x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x4 // C
	mov		w9, w5 // sdc
	lsl		w9, w9, #4 // 16*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB4
#else
	CALL(inner_scale_m11_12x4_lib4)
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
	mov		w9, w7 // sdd
	lsl		w9, w9, #4 // 16*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_L_12X4_LIB4
#else
	CALL(inner_store_l_12x4_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_spotrf_nt_l_12x4_lib4)





//                                     w0        x1         w2        x3        x4         w5       x6         w7       sp+0                sp+8    sp+16
// void kernel_spotrf_nt_l_12x4_vs_lib4(int kmax, double *A, int sda, double *B, double *C, int sdc, double *D, int sdd, double *inv_diag_D, int m1, int n1);

	.align	4
	GLOB(kernel_spotrf_nt_l_12x4_vs_lib4)
	FUN_START(kernel_spotrf_nt_l_12x4_vs_lib4)



	PROLOGUE



	ZERO_ACC



	// call inner kernel dsyrk l nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #4 // 16*sda
	mov		x11, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_12x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x4 // C
	mov		w9, w5 // sdc
	lsl		w9, w9, #4 // 16*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB4
#else
	CALL(inner_scale_m11_12x4_lib4)
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
	mov		w9, w7 // sdd
	lsl		w9, w9, #4 // 16*sdd
	ldr		w10, [sp, #(STACKSIZE + 8)] // m1
	ldr		w11, [sp, #(STACKSIZE + 16)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_12X4_VS_LIB4
#else
	CALL(inner_store_l_12x4_vs_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_spotrf_nt_l_12x4_vs_lib4)






