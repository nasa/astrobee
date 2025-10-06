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
	fmov    d11, d0; \
	fmov    d12, d0; \
	fmov    d13, d0; \
	fmov    d14, d0; \
	fmov    d15, d0

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
	fmov    d12, d0
	fmov    d13, d0
	fmov    d14, d0
	fmov    d15, d0
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
// x12  <- sdb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NT_8X8_LIB4
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_8x8_lib4)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x9, x10 // A1
	add		x14, x11, x12 // B1

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x13, #0]

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x11, #64]
	prfm	PLDL1KEEP, [x14, #64]
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x13, #64]

	// main loop
1:

	ldp		q28, q29, [x11, #(0*16)]
	ldp		q16, q17, [x14, #(0*16)]
	ldp		q24, q25, [x9, #(0*16)]
	ldp		q20, q21, [x13, #(0*16)]

	ldp		q30, q31, [x11, #(2*16)]
	ldp		q18, q19, [x14, #(2*16)]
	ldp		q26, q27, [x9, #(2*16)]
	ldp		q22, q23, [x13, #(2*16)]

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	prfm	PLDL1KEEP, [x9, #128]
	fmla	v1.4s, v24.4s, v28.s[1]
	prfm	PLDL1KEEP, [x13, #128]
	fmla	v2.4s, v24.4s, v28.s[2]
	prfm	PLDL1KEEP, [x11, #128]
	fmla	v3.4s, v24.4s, v28.s[3]
	prfm	PLDL1KEEP, [x14, #128]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v28.s[1]
	fmla	v6.4s, v20.4s, v28.s[2]
	fmla	v7.4s, v20.4s, v28.s[3]
	fmla	v8.4s, v24.4s, v16.s[0]
	fmla	v9.4s, v24.4s, v16.s[1]
	fmla	v10.4s, v24.4s, v16.s[2]
	fmla	v11.4s, v24.4s, v16.s[3]
	fmla	v12.4s, v20.4s, v16.s[0]
	fmla	v13.4s, v20.4s, v16.s[1]
	fmla	v14.4s, v20.4s, v16.s[2]
	fmla	v15.4s, v20.4s, v16.s[3]

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
	fmla	v1.4s, v25.4s, v29.s[1]
	add		x9, x9, #64
	fmla	v2.4s, v25.4s, v29.s[2]
	fmla	v3.4s, v25.4s, v29.s[3]
	add		x13, x13, #64
	fmla	v4.4s, v21.4s, v29.s[0]
	fmla	v5.4s, v21.4s, v29.s[1]
	add		x11, x11, #64
	fmla	v6.4s, v21.4s, v29.s[2]
	fmla	v7.4s, v21.4s, v29.s[3]
	add		x14, x14, #64
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
	fmla	v8.4s, v26.4s, v18.s[0]
	fmla	v9.4s, v26.4s, v18.s[1]
	fmla	v10.4s, v26.4s, v18.s[2]
	fmla	v11.4s, v26.4s, v18.s[3]
	fmla	v12.4s, v22.4s, v18.s[0]
	fmla	v13.4s, v22.4s, v18.s[1]
	fmla	v14.4s, v22.4s, v18.s[2]
	fmla	v15.4s, v22.4s, v18.s[3]
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

	ldp		q28, q29, [x11, #(0*16)]
	ldp		q16, q17, [x14, #(0*16)]
	ldp		q24, q25, [x9, #(0*16)]
	ldp		q20, q21, [x13, #(0*16)]

	ldp		q30, q31, [x11, #(2*16)]
	ldp		q18, q19, [x14, #(2*16)]
	ldp		q26, q27, [x9, #(2*16)]
	ldp		q22, q23, [x13, #(2*16)]

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v2.4s, v24.4s, v28.s[2]
	fmla	v3.4s, v24.4s, v28.s[3]
//	prfm	PLDL1KEEP, [x13, #128]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x11, #128]
	fmla	v6.4s, v20.4s, v28.s[2]
	fmla	v7.4s, v20.4s, v28.s[3]
//	prfm	PLDL1KEEP, [x14, #128]
	fmla	v8.4s, v24.4s, v16.s[0]
	fmla	v9.4s, v24.4s, v16.s[1]
	fmla	v10.4s, v24.4s, v16.s[2]
	fmla	v11.4s, v24.4s, v16.s[3]
	fmla	v12.4s, v20.4s, v16.s[0]
	fmla	v13.4s, v20.4s, v16.s[1]
	fmla	v14.4s, v20.4s, v16.s[2]
	fmla	v15.4s, v20.4s, v16.s[3]

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
	fmla	v1.4s, v25.4s, v29.s[1]
	add		x9, x9, #64
	fmla	v2.4s, v25.4s, v29.s[2]
	fmla	v3.4s, v25.4s, v29.s[3]
	add		x13, x13, #64
	fmla	v4.4s, v21.4s, v29.s[0]
	fmla	v5.4s, v21.4s, v29.s[1]
	add		x11, x11, #64
	fmla	v6.4s, v21.4s, v29.s[2]
	fmla	v7.4s, v21.4s, v29.s[3]
	add		x14, x14, #64
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
	fmla	v8.4s, v26.4s, v18.s[0]
	fmla	v9.4s, v26.4s, v18.s[1]
	fmla	v10.4s, v26.4s, v18.s[2]
	fmla	v11.4s, v26.4s, v18.s[3]
	fmla	v12.4s, v22.4s, v18.s[0]
	fmla	v13.4s, v22.4s, v18.s[1]
	fmla	v14.4s, v22.4s, v18.s[2]
	fmla	v15.4s, v22.4s, v18.s[3]
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
//	sub		x13, x13, #32
//	sub		x11, x11, #32
//	sub		x14, x14, #32

3: // clean1-up loop

	// unroll 0

	ld1		{v28.4s}, [x11], #16
	ld1		{v24.4s}, [x9], #16
	ld1		{v20.4s}, [x13], #16
	ld1		{v16.4s}, [x14], #16
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v28.s[1]
	fmla	v2.4s, v24.4s, v28.s[2]
	fmla	v3.4s, v24.4s, v28.s[3]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v28.s[1]
	fmla	v6.4s, v20.4s, v28.s[2]
	fmla	v7.4s, v20.4s, v28.s[3]
	fmla	v8.4s, v24.4s, v16.s[0]
	fmla	v9.4s, v24.4s, v16.s[1]
	fmla	v10.4s, v24.4s, v16.s[2]
	fmla	v11.4s, v24.4s, v16.s[3]
	fmla	v12.4s, v20.4s, v16.s[0]
	fmla	v13.4s, v20.4s, v16.s[1]
	fmla	v14.4s, v20.4s, v16.s[2]
	fmla	v15.4s, v20.4s, v16.s[3]

	sub		w8, w8, #1
	cmp		w8, #0
	bgt		3b

2: // return



#else // cortex a53



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x9, x10 // A1
	add		x14, x11, x12 // B1

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x13, #0]

	// preload
	ldp		q28, q29, [x11, #(0*16)]
	ldp		q30, q31, [x11, #(2*16)]
	ldp		q16, q17, [x14, #(0*16)]
	ldp		q18, q19, [x14, #(2*16)]

	ldp		q24, q25, [x9, #(0*16)]
	ldp		q20, q21, [x13, #(0*16)]

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x11, #64]
	prfm	PLDL1KEEP, [x14, #64]
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x13, #64]

	// main loop
1:

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v28.s[1]
	ldp		q26, q27, [x9, #(2*16)]
	fmla	v2.4s, v24.4s, v28.s[2]
	fmla	v3.4s, v24.4s, v28.s[3]
	ldp		q22, q23, [x13, #(2*16)]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v28.s[1]
	prfm	PLDL1KEEP, [x9, #128]
	fmla	v6.4s, v20.4s, v28.s[2]
	fmla	v7.4s, v20.4s, v28.s[3]
	prfm	PLDL1KEEP, [x13, #128]
	fmla	v8.4s, v24.4s, v16.s[0]
	fmla	v9.4s, v24.4s, v16.s[1]
	prfm	PLDL1KEEP, [x11, #128]
	fmla	v10.4s, v24.4s, v16.s[2]
	fmla	v11.4s, v24.4s, v16.s[3]
	prfm	PLDL1KEEP, [x14, #128]
	fmla	v12.4s, v20.4s, v16.s[0]
	fmla	v13.4s, v20.4s, v16.s[1]
	fmla	v14.4s, v20.4s, v16.s[2]
	fmla	v15.4s, v20.4s, v16.s[3]

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
	fmla	v1.4s, v25.4s, v29.s[1]
	add		x9, x9, #64
	fmla	v2.4s, v25.4s, v29.s[2]
	fmla	v3.4s, v25.4s, v29.s[3]
	add		x13, x13, #64
	fmla	v4.4s, v21.4s, v29.s[0]
	fmla	v5.4s, v21.4s, v29.s[1]
	add		x11, x11, #64
	fmla	v6.4s, v21.4s, v29.s[2]
	fmla	v7.4s, v21.4s, v29.s[3]
	add		x14, x14, #64
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
	ldp		q24, q25, [x9, #(0*16)]
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v3.4s, v26.4s, v30.s[3]
	ldp		q20, q21, [x13, #(0*16)]
	fmla	v4.4s, v22.4s, v30.s[0]
	fmla	v5.4s, v22.4s, v30.s[1]
	sub		w8, w8, #4
	fmla	v6.4s, v22.4s, v30.s[2]
	fmla	v7.4s, v22.4s, v30.s[3]
	fmla	v8.4s, v26.4s, v18.s[0]
	fmla	v9.4s, v26.4s, v18.s[1]
	fmla	v10.4s, v26.4s, v18.s[2]
	fmla	v11.4s, v26.4s, v18.s[3]
	fmla	v12.4s, v22.4s, v18.s[0]
	fmla	v13.4s, v22.4s, v18.s[1]
	fmla	v14.4s, v22.4s, v18.s[2]
	fmla	v15.4s, v22.4s, v18.s[3]

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
	fmla	v1.4s, v27.4s, v31.s[1]
	ldp		q28, q29, [x11, #(0*16)]
	fmla	v2.4s, v27.4s, v31.s[2]
	fmla	v3.4s, v27.4s, v31.s[3]
	ldp		q16, q17, [x14, #(0*16)]
	fmla	v4.4s, v23.4s, v31.s[0]
	fmla	v5.4s, v23.4s, v31.s[1]
	fmla	v6.4s, v23.4s, v31.s[2]
	fmla	v7.4s, v23.4s, v31.s[3]
	ldp		q30, q31, [x11, #(2*16)]
	fmla	v8.4s, v27.4s, v19.s[0]
	fmla	v9.4s, v27.4s, v19.s[1]
	fmla	v10.4s, v27.4s, v19.s[2]
	fmla	v11.4s, v27.4s, v19.s[3]
	fmla	v12.4s, v23.4s, v19.s[0]
	fmla	v13.4s, v23.4s, v19.s[1]
	fmla	v14.4s, v23.4s, v19.s[2]
	fmla	v15.4s, v23.4s, v19.s[3]
	ldp		q18, q19, [x14, #(2*16)]

	cmp		w8, #4
	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v28.s[1]
	ldp		q26, q27, [x9, #(2*16)]
	fmla	v2.4s, v24.4s, v28.s[2]
	fmla	v3.4s, v24.4s, v28.s[3]
	ldp		q22, q23, [x13, #(2*16)]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x9, #64]
	fmla	v6.4s, v20.4s, v28.s[2]
	fmla	v7.4s, v20.4s, v28.s[3]
//	prfm	PLDL1KEEP, [x13, #64]
	fmla	v8.4s, v24.4s, v16.s[0]
	fmla	v9.4s, v24.4s, v16.s[1]
//	prfm	PLDL1KEEP, [x11, #64]
	fmla	v10.4s, v24.4s, v16.s[2]
	fmla	v11.4s, v24.4s, v16.s[3]
//	prfm	PLDL1KEEP, [x14, #64]
	fmla	v12.4s, v20.4s, v16.s[0]
	fmla	v13.4s, v20.4s, v16.s[1]
	fmla	v14.4s, v20.4s, v16.s[2]
	fmla	v15.4s, v20.4s, v16.s[3]

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
	fmla	v1.4s, v25.4s, v29.s[1]
	add		x9, x9, #64
	fmla	v2.4s, v25.4s, v29.s[2]
	fmla	v3.4s, v25.4s, v29.s[3]
	add		x13, x13, #64
	fmla	v4.4s, v21.4s, v29.s[0]
	fmla	v5.4s, v21.4s, v29.s[1]
	add		x11, x11, #64
	fmla	v6.4s, v21.4s, v29.s[2]
	fmla	v7.4s, v21.4s, v29.s[3]
	add		x14, x14, #64
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
//	ldp		q24, q25, [x9, #(0*16)]
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v3.4s, v26.4s, v30.s[3]
//	ldp		q20, q21, [x13, #(0*16)]
	fmla	v4.4s, v22.4s, v30.s[0]
	fmla	v5.4s, v22.4s, v30.s[1]
	sub		w8, w8, #4
	fmla	v6.4s, v22.4s, v30.s[2]
	fmla	v7.4s, v22.4s, v30.s[3]
	fmla	v8.4s, v26.4s, v18.s[0]
	fmla	v9.4s, v26.4s, v18.s[1]
	fmla	v10.4s, v26.4s, v18.s[2]
	fmla	v11.4s, v26.4s, v18.s[3]
	fmla	v12.4s, v22.4s, v18.s[0]
	fmla	v13.4s, v22.4s, v18.s[1]
	fmla	v14.4s, v22.4s, v18.s[2]
	fmla	v15.4s, v22.4s, v18.s[3]

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
	fmla	v1.4s, v27.4s, v31.s[1]
//	ldp		q28, q29, [x11, #(0*16)]
	fmla	v2.4s, v27.4s, v31.s[2]
	fmla	v3.4s, v27.4s, v31.s[3]
//	ldp		q16, q17, [x14, #(0*16)]
	fmla	v4.4s, v23.4s, v31.s[0]
	fmla	v5.4s, v23.4s, v31.s[1]
	fmla	v6.4s, v23.4s, v31.s[2]
	fmla	v7.4s, v23.4s, v31.s[3]
//	ldp		q30, q31, [x11, #(2*16)]
	fmla	v8.4s, v27.4s, v19.s[0]
	fmla	v9.4s, v27.4s, v19.s[1]
	fmla	v10.4s, v27.4s, v19.s[2]
	fmla	v11.4s, v27.4s, v19.s[3]
	fmla	v12.4s, v23.4s, v19.s[0]
	fmla	v13.4s, v23.4s, v19.s[1]
	fmla	v14.4s, v23.4s, v19.s[2]
	fmla	v15.4s, v23.4s, v19.s[3]
//	ldp		q18, q19, [x14, #(2*16)]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x9, x9, #32
//	sub		x13, x13, #32
//	sub		x11, x11, #32
//	sub		x14, x14, #32

3: // clean1-up loop

	// unroll 0

	ld1		{v28.4s}, [x11], #16
	ld1		{v24.4s}, [x9], #16
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v28.s[1]
	fmla	v2.4s, v24.4s, v28.s[2]
	fmla	v3.4s, v24.4s, v28.s[3]
	ld1		{v20.4s}, [x13], #16
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v28.s[1]
	fmla	v6.4s, v20.4s, v28.s[2]
	fmla	v7.4s, v20.4s, v28.s[3]
	ld1		{v16.4s}, [x14], #16
	fmla	v8.4s, v24.4s, v16.s[0]
	fmla	v9.4s, v24.4s, v16.s[1]
	fmla	v10.4s, v24.4s, v16.s[2]
	fmla	v11.4s, v24.4s, v16.s[3]
	fmla	v12.4s, v20.4s, v16.s[0]
	fmla	v13.4s, v20.4s, v16.s[1]
	fmla	v14.4s, v20.4s, v16.s[2]
	fmla	v15.4s, v20.4s, v16.s[3]

	sub		w8, w8, #1
	cmp		w8, #0
	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_8x8_lib4)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- sda
// x11  <- B
// x12  <- sdb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_8X8_LIB4
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_8x8_lib4)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x9, x10 // A1
	add		x14, x11, #64 // B1

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x13, #0]

	// preload


	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x14, x12]
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x13, #64]

	// main loop
1:

	ldp		q24, q25, [x9, #(0*16)]
	ldp		q20, q21, [x13, #(0*16)]

	ldp		q28, q29, [x11, #(0*16)]
	ldp		q30, q31, [x11, #(2*16)]

	ldp		q16, q17, [x14, #(0*16)]
	ldp		q18, q19, [x14, #(2*16)]

	ldp		q26, q27, [x9, #(2*16)]
	ldp		q22, q23, [x13, #(2*16)]

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v4.4s, v20.4s, v28.s[0]
	prfm	PLDL1KEEP, [x9, #128]
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v5.4s, v20.4s, v29.s[0]
	prfm	PLDL1KEEP, [x13, #128]
	fmla	v2.4s, v24.4s, v30.s[0]
	fmla	v6.4s, v20.4s, v30.s[0]
	add		x11, x11, x12
	fmla	v3.4s, v24.4s, v31.s[0]
	fmla	v7.4s, v20.4s, v31.s[0]
	prfm	PLDL1KEEP, [x11, x12]
	fmla	v8.4s, v24.4s, v16.s[0]
	fmla	v12.4s, v20.4s, v16.s[0]
	add		x14, x14, x12
	fmla	v9.4s, v24.4s, v17.s[0]
	fmla	v13.4s, v20.4s, v17.s[0]
	prfm	PLDL1KEEP, [x14, x12]
	fmla	v10.4s, v24.4s, v18.s[0]
	fmla	v14.4s, v20.4s, v18.s[0]
	add		x9, x9, #64
	fmla	v11.4s, v24.4s, v19.s[0]
	fmla	v15.4s, v20.4s, v19.s[0]
	add		x13, x13, #64

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
	fmla	v4.4s, v21.4s, v28.s[1]
	fmla	v1.4s, v25.4s, v29.s[1]
	fmla	v5.4s, v21.4s, v29.s[1]
	fmla	v2.4s, v25.4s, v30.s[1]
	fmla	v6.4s, v21.4s, v30.s[1]
	fmla	v3.4s, v25.4s, v31.s[1]
	fmla	v7.4s, v21.4s, v31.s[1]
	fmla	v8.4s, v25.4s, v16.s[1]
	fmla	v12.4s, v21.4s, v16.s[1]
	fmla	v9.4s, v25.4s, v17.s[1]
	fmla	v13.4s, v21.4s, v17.s[1]
	fmla	v10.4s, v25.4s, v18.s[1]
	fmla	v14.4s, v21.4s, v18.s[1]
	fmla	v11.4s, v25.4s, v19.s[1]
	fmla	v15.4s, v21.4s, v19.s[1]

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v4.4s, v22.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
	fmla	v5.4s, v22.4s, v29.s[2]
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v6.4s, v22.4s, v30.s[2]
	sub		w8, w8, #4
	fmla	v3.4s, v26.4s, v31.s[2]
	fmla	v7.4s, v22.4s, v31.s[2]
	fmla	v8.4s, v26.4s, v16.s[2]
	fmla	v12.4s, v22.4s, v16.s[2]
	fmla	v9.4s, v26.4s, v17.s[2]
	fmla	v13.4s, v22.4s, v17.s[2]
	fmla	v10.4s, v26.4s, v18.s[2]
	fmla	v14.4s, v22.4s, v18.s[2]
	fmla	v11.4s, v26.4s, v19.s[2]
	fmla	v15.4s, v22.4s, v19.s[2]

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
	fmla	v4.4s, v23.4s, v28.s[3]
	fmla	v1.4s, v27.4s, v29.s[3]
	fmla	v5.4s, v23.4s, v29.s[3]
	fmla	v2.4s, v27.4s, v30.s[3]
	fmla	v6.4s, v23.4s, v30.s[3]
	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v7.4s, v23.4s, v31.s[3]
	fmla	v8.4s, v27.4s, v16.s[3]
	fmla	v12.4s, v23.4s, v16.s[3]
	fmla	v9.4s, v27.4s, v17.s[3]
	fmla	v13.4s, v23.4s, v17.s[3]
	fmla	v10.4s, v27.4s, v18.s[3]
	fmla	v14.4s, v23.4s, v18.s[3]
	fmla	v11.4s, v27.4s, v19.s[3]
	fmla	v15.4s, v23.4s, v19.s[3]

	cmp		w8, #4
	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	ldp		q28, q29, [x11, #(0*16)]
	ldp		q30, q31, [x11, #(2*16)]
	ldp		q16, q17, [x14, #(0*16)]
	ldp		q18, q19, [x14, #(2*16)]

	ldp		q24, q25, [x9, #(0*16)]
	ldp		q20, q21, [x13, #(0*16)]
	ldp		q26, q27, [x9, #(2*16)]
	ldp		q22, q23, [x13, #(2*16)]

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v5.4s, v20.4s, v29.s[0]
	fmla	v2.4s, v24.4s, v30.s[0]
	fmla	v6.4s, v20.4s, v30.s[0]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v3.4s, v24.4s, v31.s[0]
	fmla	v7.4s, v20.4s, v31.s[0]
//	prfm	PLDL1KEEP, [x13, #128]
	fmla	v8.4s, v24.4s, v16.s[0]
	fmla	v12.4s, v20.4s, v16.s[0]
	add		x11, x11, x12
	fmla	v9.4s, v24.4s, v17.s[0]
	fmla	v13.4s, v20.4s, v17.s[0]
//	prfm	PLDL1KEEP, [x11, x12]
	fmla	v10.4s, v24.4s, v18.s[0]
	fmla	v14.4s, v20.4s, v18.s[0]
	add		x14, x14, x12
	fmla	v11.4s, v24.4s, v19.s[0]
	fmla	v15.4s, v20.4s, v19.s[0]
//	prfm	PLDL1KEEP, [x14, x12]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
	fmla	v4.4s, v21.4s, v28.s[1]
	add		x9, x9, #64
	fmla	v1.4s, v25.4s, v29.s[1]
	fmla	v5.4s, v21.4s, v29.s[1]
	add		x13, x13, #64
	fmla	v2.4s, v25.4s, v30.s[1]
	fmla	v6.4s, v21.4s, v30.s[1]
	fmla	v3.4s, v25.4s, v31.s[1]
	fmla	v7.4s, v21.4s, v31.s[1]
	fmla	v8.4s, v25.4s, v16.s[1]
	fmla	v12.4s, v21.4s, v16.s[1]
	fmla	v9.4s, v25.4s, v17.s[1]
	fmla	v13.4s, v21.4s, v17.s[1]
	fmla	v10.4s, v25.4s, v18.s[1]
	fmla	v14.4s, v21.4s, v18.s[1]
	fmla	v11.4s, v25.4s, v19.s[1]
	fmla	v15.4s, v21.4s, v19.s[1]

//	ldp		q24, q25, [x9, #(0*16)]
//	ldp		q20, q21, [x13, #(0*16)]

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v4.4s, v22.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
	fmla	v5.4s, v22.4s, v29.s[2]
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v6.4s, v22.4s, v30.s[2]
	sub		w8, w8, #4
	fmla	v3.4s, v26.4s, v31.s[2]
	fmla	v7.4s, v22.4s, v31.s[2]
	fmla	v8.4s, v26.4s, v16.s[2]
	fmla	v12.4s, v22.4s, v16.s[2]
	fmla	v9.4s, v26.4s, v17.s[2]
	fmla	v13.4s, v22.4s, v17.s[2]
	fmla	v10.4s, v26.4s, v18.s[2]
	fmla	v14.4s, v22.4s, v18.s[2]
	fmla	v11.4s, v26.4s, v19.s[2]
	fmla	v15.4s, v22.4s, v19.s[2]

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
	fmla	v4.4s, v23.4s, v28.s[3]
	fmla	v1.4s, v27.4s, v29.s[3]
	fmla	v5.4s, v23.4s, v29.s[3]
	fmla	v2.4s, v27.4s, v30.s[3]
	fmla	v6.4s, v23.4s, v30.s[3]
	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v7.4s, v23.4s, v31.s[3]
	fmla	v8.4s, v27.4s, v16.s[3]
	fmla	v12.4s, v23.4s, v16.s[3]
	fmla	v9.4s, v27.4s, v17.s[3]
	fmla	v13.4s, v23.4s, v17.s[3]
	fmla	v10.4s, v27.4s, v18.s[3]
	fmla	v14.4s, v23.4s, v18.s[3]
	fmla	v11.4s, v27.4s, v19.s[3]
	fmla	v15.4s, v23.4s, v19.s[3]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x9, x9, #32
//	sub		x13, x13, #32
//	sub		x11, x11, #32
//	sub		x14, x14, #32

3: // clean1-up loop

	// unroll 0

	ldr		s28, [x11, #(0*16)]
	ldr		s29, [x11, #(1*16)]
	ldr		s30, [x11, #(2*16)]
	ldr		s31, [x11, #(3*16)]
	ldr		s16, [x14, #(0*16)]
	ldr		s17, [x14, #(1*16)]
	ldr		s18, [x14, #(2*16)]
	ldr		s19, [x14, #(3*16)]
	ld1		{v24.4s}, [x9], #16
	ld1		{v20.4s}, [x13], #16
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v5.4s, v20.4s, v29.s[0]
	fmla	v2.4s, v24.4s, v30.s[0]
	fmla	v6.4s, v20.4s, v30.s[0]
	fmla	v3.4s, v24.4s, v31.s[0]
	fmla	v7.4s, v20.4s, v31.s[0]
	fmla	v8.4s, v24.4s, v16.s[0]
	fmla	v12.4s, v20.4s, v16.s[0]
	fmla	v9.4s, v24.4s, v17.s[0]
	fmla	v13.4s, v20.4s, v17.s[0]
	fmla	v10.4s, v24.4s, v18.s[0]
	fmla	v14.4s, v20.4s, v18.s[0]
	fmla	v11.4s, v24.4s, v19.s[0]
	fmla	v15.4s, v20.4s, v19.s[0]

	add		x11, x11, #4
	add		x14, x14, #4

	sub		w8, w8, #1
	cmp		w8, #0
	bgt		3b

2: // return



#else // cortex a53



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x9, x10 // A1
	add		x14, x11, #64 // B1

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x13, #0]

	// preload
	ldp		q28, q29, [x11, #(0*16)]
	ldp		q30, q31, [x11, #(2*16)]
	ldp		q16, q17, [x14, #(0*16)]
	ldp		q18, q19, [x14, #(2*16)]

	ldp		q24, q25, [x9, #(0*16)]
	ldp		q20, q21, [x13, #(0*16)]

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x14, x12]
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x13, #64]

	// main loop
1:

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v4.4s, v20.4s, v28.s[0]
	ldp		q26, q27, [x9, #(2*16)]
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v5.4s, v20.4s, v29.s[0]
	ldp		q22, q23, [x13, #(2*16)]
	fmla	v2.4s, v24.4s, v30.s[0]
	fmla	v6.4s, v20.4s, v30.s[0]
	prfm	PLDL1KEEP, [x9, #128]
	fmla	v3.4s, v24.4s, v31.s[0]
	fmla	v7.4s, v20.4s, v31.s[0]
	prfm	PLDL1KEEP, [x13, #128]
	fmla	v8.4s, v24.4s, v16.s[0]
	fmla	v12.4s, v20.4s, v16.s[0]
	add		x11, x11, x12
	fmla	v9.4s, v24.4s, v17.s[0]
	fmla	v13.4s, v20.4s, v17.s[0]
	prfm	PLDL1KEEP, [x11, x12]
	fmla	v10.4s, v24.4s, v18.s[0]
	fmla	v14.4s, v20.4s, v18.s[0]
	add		x14, x14, x12
	fmla	v11.4s, v24.4s, v19.s[0]
	fmla	v15.4s, v20.4s, v19.s[0]
	prfm	PLDL1KEEP, [x14, x12]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
	fmla	v4.4s, v21.4s, v28.s[1]
	add		x9, x9, #64
	fmla	v1.4s, v25.4s, v29.s[1]
	fmla	v5.4s, v21.4s, v29.s[1]
	add		x13, x13, #64
	fmla	v2.4s, v25.4s, v30.s[1]
	fmla	v6.4s, v21.4s, v30.s[1]
	fmla	v3.4s, v25.4s, v31.s[1]
	fmla	v7.4s, v21.4s, v31.s[1]
	fmla	v8.4s, v25.4s, v16.s[1]
	fmla	v12.4s, v21.4s, v16.s[1]
	fmla	v9.4s, v25.4s, v17.s[1]
	fmla	v13.4s, v21.4s, v17.s[1]
	fmla	v10.4s, v25.4s, v18.s[1]
	fmla	v14.4s, v21.4s, v18.s[1]
	fmla	v11.4s, v25.4s, v19.s[1]
	fmla	v15.4s, v21.4s, v19.s[1]

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v4.4s, v22.4s, v28.s[2]
	ldp		q24, q25, [x9, #(0*16)]
	fmla	v1.4s, v26.4s, v29.s[2]
	fmla	v5.4s, v22.4s, v29.s[2]
	ldp		q20, q21, [x13, #(0*16)]
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v6.4s, v22.4s, v30.s[2]
	sub		w8, w8, #4
	fmla	v3.4s, v26.4s, v31.s[2]
	fmla	v7.4s, v22.4s, v31.s[2]
	fmla	v8.4s, v26.4s, v16.s[2]
	fmla	v12.4s, v22.4s, v16.s[2]
	fmla	v9.4s, v26.4s, v17.s[2]
	fmla	v13.4s, v22.4s, v17.s[2]
	fmla	v10.4s, v26.4s, v18.s[2]
	fmla	v14.4s, v22.4s, v18.s[2]
	fmla	v11.4s, v26.4s, v19.s[2]
	fmla	v15.4s, v22.4s, v19.s[2]

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
	fmla	v4.4s, v23.4s, v28.s[3]
	fmla	v1.4s, v27.4s, v29.s[3]
	fmla	v5.4s, v23.4s, v29.s[3]
	ldp		q28, q29, [x11, #(0*16)]
	fmla	v2.4s, v27.4s, v30.s[3]
	fmla	v6.4s, v23.4s, v30.s[3]
	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v7.4s, v23.4s, v31.s[3]
	ldp		q30, q31, [x11, #(2*16)]
	fmla	v8.4s, v27.4s, v16.s[3]
	fmla	v12.4s, v23.4s, v16.s[3]
	fmla	v9.4s, v27.4s, v17.s[3]
	fmla	v13.4s, v23.4s, v17.s[3]
	ldp		q16, q17, [x14, #(0*16)]
	fmla	v10.4s, v27.4s, v18.s[3]
	fmla	v14.4s, v23.4s, v18.s[3]
	fmla	v11.4s, v27.4s, v19.s[3]
	fmla	v15.4s, v23.4s, v19.s[3]
	ldp		q18, q19, [x14, #(2*16)]

	cmp		w8, #4
	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v4.4s, v20.4s, v28.s[0]
	ldp		q26, q27, [x9, #(2*16)]
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v5.4s, v20.4s, v29.s[0]
	ldp		q22, q23, [x13, #(2*16)]
	fmla	v2.4s, v24.4s, v30.s[0]
	fmla	v6.4s, v20.4s, v30.s[0]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v3.4s, v24.4s, v31.s[0]
	fmla	v7.4s, v20.4s, v31.s[0]
//	prfm	PLDL1KEEP, [x13, #128]
	fmla	v8.4s, v24.4s, v16.s[0]
	fmla	v12.4s, v20.4s, v16.s[0]
	add		x11, x11, x12
	fmla	v9.4s, v24.4s, v17.s[0]
	fmla	v13.4s, v20.4s, v17.s[0]
//	prfm	PLDL1KEEP, [x11, x12]
	fmla	v10.4s, v24.4s, v18.s[0]
	fmla	v14.4s, v20.4s, v18.s[0]
	add		x14, x14, x12
	fmla	v11.4s, v24.4s, v19.s[0]
	fmla	v15.4s, v20.4s, v19.s[0]
//	prfm	PLDL1KEEP, [x14, x12]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
	fmla	v4.4s, v21.4s, v28.s[1]
	add		x9, x9, #64
	fmla	v1.4s, v25.4s, v29.s[1]
	fmla	v5.4s, v21.4s, v29.s[1]
	add		x13, x13, #64
	fmla	v2.4s, v25.4s, v30.s[1]
	fmla	v6.4s, v21.4s, v30.s[1]
	fmla	v3.4s, v25.4s, v31.s[1]
	fmla	v7.4s, v21.4s, v31.s[1]
	fmla	v8.4s, v25.4s, v16.s[1]
	fmla	v12.4s, v21.4s, v16.s[1]
	fmla	v9.4s, v25.4s, v17.s[1]
	fmla	v13.4s, v21.4s, v17.s[1]
	fmla	v10.4s, v25.4s, v18.s[1]
	fmla	v14.4s, v21.4s, v18.s[1]
	fmla	v11.4s, v25.4s, v19.s[1]
	fmla	v15.4s, v21.4s, v19.s[1]

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v4.4s, v22.4s, v28.s[2]
//	ldp		q24, q25, [x9, #(0*16)]
	fmla	v1.4s, v26.4s, v29.s[2]
	fmla	v5.4s, v22.4s, v29.s[2]
//	ldp		q20, q21, [x13, #(0*16)]
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v6.4s, v22.4s, v30.s[2]
	sub		w8, w8, #4
	fmla	v3.4s, v26.4s, v31.s[2]
	fmla	v7.4s, v22.4s, v31.s[2]
	fmla	v8.4s, v26.4s, v16.s[2]
	fmla	v12.4s, v22.4s, v16.s[2]
	fmla	v9.4s, v26.4s, v17.s[2]
	fmla	v13.4s, v22.4s, v17.s[2]
	fmla	v10.4s, v26.4s, v18.s[2]
	fmla	v14.4s, v22.4s, v18.s[2]
	fmla	v11.4s, v26.4s, v19.s[2]
	fmla	v15.4s, v22.4s, v19.s[2]

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
	fmla	v4.4s, v23.4s, v28.s[3]
	fmla	v1.4s, v27.4s, v29.s[3]
	fmla	v5.4s, v23.4s, v29.s[3]
//	ldp		q28, q29, [x11, #(0*16)]
	fmla	v2.4s, v27.4s, v30.s[3]
	fmla	v6.4s, v23.4s, v30.s[3]
	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v7.4s, v23.4s, v31.s[3]
//	ldp		q30, q31, [x11, #(2*16)]
	fmla	v8.4s, v27.4s, v16.s[3]
	fmla	v12.4s, v23.4s, v16.s[3]
	fmla	v9.4s, v27.4s, v17.s[3]
	fmla	v13.4s, v23.4s, v17.s[3]
//	ldp		q16, q17, [x14, #(0*16)]
	fmla	v10.4s, v27.4s, v18.s[3]
	fmla	v14.4s, v23.4s, v18.s[3]
	fmla	v11.4s, v27.4s, v19.s[3]
	fmla	v15.4s, v23.4s, v19.s[3]
//	ldp		q18, q19, [x14, #(2*16)]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x9, x9, #32
//	sub		x13, x13, #32
//	sub		x11, x11, #32
//	sub		x14, x14, #32

3: // clean1-up loop

	// unroll 0

	ldr		s28, [x11, #(0*16)]
	ldr		s29, [x11, #(1*16)]
	ldr		s30, [x11, #(2*16)]
	ldr		s31, [x11, #(3*16)]
	ldr		s16, [x14, #(0*16)]
	ldr		s17, [x14, #(1*16)]
	ldr		s18, [x14, #(2*16)]
	ldr		s19, [x14, #(3*16)]
	ld1		{v24.4s}, [x9], #16
	ld1		{v20.4s}, [x13], #16
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v5.4s, v20.4s, v29.s[0]
	fmla	v2.4s, v24.4s, v30.s[0]
	fmla	v6.4s, v20.4s, v30.s[0]
	fmla	v3.4s, v24.4s, v31.s[0]
	fmla	v7.4s, v20.4s, v31.s[0]
	fmla	v8.4s, v24.4s, v16.s[0]
	fmla	v12.4s, v20.4s, v16.s[0]
	fmla	v9.4s, v24.4s, v17.s[0]
	fmla	v13.4s, v20.4s, v17.s[0]
	fmla	v10.4s, v24.4s, v18.s[0]
	fmla	v14.4s, v20.4s, v18.s[0]
	fmla	v11.4s, v24.4s, v19.s[0]
	fmla	v15.4s, v20.4s, v19.s[0]

	add		x11, x11, #4
	add		x14, x14, #4

	sub		w8, w8, #1
	cmp		w8, #0
	bgt		3b

2: // return



#endif



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_8x8_lib4)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- sda
// x11   <- B
// x12   <- 16*sdb
// w13   <- offsetB

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_GEMM_ADD_NN_8X8_LIB4
#else
	.align	4
	FUN_START(inner_edge_gemm_add_nn_8x8_lib4)
#endif

	cmp		w13, #0
	ble		2f // return

	cmp		w8, #0
	ble		2f // return

	mov		w14, #4
	sub		w15, w14, w13 // 4-offsetB
	cmp		w15, w8
	ble		0f
	mov		w15, w8 // kend=min(k,4-offsetB)
0:
//	movgt	w15, w8 // kend=min(k,4-offsetB)
	
	add		x11, x11, x13, LSL #2 // B + offsetB*sizeof(float)

	add		x14, x9, x10

1:
	ldr		s28, [x11, #(0*16)]
	ldr		s29, [x11, #(1*16)]
	ldr		s30, [x11, #(2*16)]
	ldr		s31, [x11, #(3*16)]
	ldr		s16, [x11, #(4*16)]
	ldr		s17, [x11, #(5*16)]
	ldr		s18, [x11, #(6*16)]
	ldr		s19, [x11, #(7*16)]
	ld1		{v24.4s}, [x9], #16
	ld1		{v20.4s}, [x14], #16
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v5.4s, v20.4s, v29.s[0]
	fmla	v2.4s, v24.4s, v30.s[0]
	fmla	v6.4s, v20.4s, v30.s[0]
	fmla	v3.4s, v24.4s, v31.s[0]
	fmla	v7.4s, v20.4s, v31.s[0]
	fmla	v8.4s, v24.4s, v16.s[0]
	fmla	v12.4s, v20.4s, v16.s[0]
	fmla	v9.4s, v24.4s, v17.s[0]
	fmla	v13.4s, v20.4s, v17.s[0]
	fmla	v10.4s, v24.4s, v18.s[0]
	fmla	v14.4s, v20.4s, v18.s[0]
	fmla	v11.4s, v24.4s, v19.s[0]
	fmla	v15.4s, v20.4s, v19.s[0]

	add		x11, x11, #4

	sub		w8, w8, #1

	sub		w15, w15, #1

	cmp		w15, #0
	bgt		1b

	cmp		w8, #0
	ble		2f // return

	add		x11, x11, x12
	sub		x11, x11, #16

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_gemm_add_nn_8x8_lib4)
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
	.macro INNER_SCALE_AB_8X8_LIB4
#else
	.align	4
	FUN_START(inner_scale_ab_8x8_lib4)
#endif

	ld1		{v28.4s}, [x8]

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

	ld1		{v28.4s}, [x9]

	add		x12, x10, x11

	ld1		{v24.4s, v25.4s, v26.4s, v27.4s}, [x10], #64
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v25.4s, v28.s[0]
	fmla	v2.4s, v26.4s, v28.s[0]
	fmla	v3.4s, v27.4s, v28.s[0]

	ld1		{v24.4s, v25.4s, v26.4s, v27.4s}, [x12], #64
	fmla	v4.4s, v24.4s, v28.s[0]
	fmla	v5.4s, v25.4s, v28.s[0]
	fmla	v6.4s, v26.4s, v28.s[0]
	fmla	v7.4s, v27.4s, v28.s[0]

	ld1		{v24.4s, v25.4s, v26.4s, v27.4s}, [x10], #64
	fmla	v8.4s, v24.4s, v28.s[0]
	fmla	v9.4s, v25.4s, v28.s[0]
	fmla	v10.4s, v26.4s, v28.s[0]
	fmla	v11.4s, v27.4s, v28.s[0]

	ld1		{v24.4s, v25.4s, v26.4s, v27.4s}, [x12], #64
	fmla	v12.4s, v24.4s, v28.s[0]
	fmla	v13.4s, v25.4s, v28.s[0]
	fmla	v14.4s, v26.4s, v28.s[0]
	fmla	v15.4s, v27.4s, v28.s[0]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_8x8_lib4)
#endif





// subroutine
//
// input arguments:
// x8   <- D
// x9   <- sdd
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_8X8_LIB4
#else
	.align 4
	FUN_START(inner_store_8x8_lib4)
#endif

	add		x10, x8, x9

	stp		q0, q1, [x8, #0]
	stp		q2, q3, [x8, #32]
	stp		q8, q9, [x8, #64]
	stp		q10, q11, [x8, #96]

	stp		q4, q5, [x10, #0]
	stp		q6, q7, [x10, #32]
	stp		q12, q13, [x10, #64]
	stp		q14, q15, [x10, #96]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_8x8_lib4)
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
	.macro INNER_STORE_8X8_VS_LIB4
#else
	.align 4
	FUN_START(inner_store_8x8_vs_lib4)
#endif

	add		x12, x8, x9

	cmp		w10, #8
	bge		1f

	ldp		q24, q25, [x12, #(0*16)]
	ldp		q26, q27, [x12, #(2*16)]
	ldp		q28, q29, [x12, #(4*16)]
	ldp		q30, q31, [x12, #(6*16)]

	// 4th row
	ins		v4.s[3], v24.s[3]
	ins		v5.s[3], v25.s[3]
	ins		v6.s[3], v26.s[3]
	ins		v7.s[3], v27.s[3]
	ins		v12.s[3], v28.s[3]
	ins		v13.s[3], v29.s[3]
	ins		v14.s[3], v30.s[3]
	ins		v15.s[3], v31.s[3]
	cmp		w10, #7
	bge		1f
	// 3th row
	ins		v4.s[2], v24.s[2]
	ins		v5.s[2], v25.s[2]
	ins		v6.s[2], v26.s[2]
	ins		v7.s[2], v27.s[2]
	ins		v12.s[2], v28.s[2]
	ins		v13.s[2], v29.s[2]
	ins		v14.s[2], v30.s[2]
	ins		v15.s[2], v31.s[2]
	cmp		w10, #6
	bge		1f
	// 2nd row
	ins		v4.s[1], v24.s[1]
	ins		v5.s[1], v25.s[1]
	ins		v6.s[1], v26.s[1]
	ins		v7.s[1], v27.s[1]
	ins		v12.s[1], v28.s[1]
	ins		v13.s[1], v29.s[1]
	ins		v14.s[1], v30.s[1]
	ins		v15.s[1], v31.s[1]
	cmp		w10, #5
	bge		1f
	// 1st row
	ins		v4.s[0], v24.s[0]
	ins		v5.s[0], v25.s[0]
	ins		v6.s[0], v26.s[0]
	ins		v7.s[0], v27.s[0]
	ins		v12.s[0], v28.s[0]
	ins		v13.s[0], v29.s[0]
	ins		v14.s[0], v30.s[0]
	ins		v15.s[0], v31.s[0]

1:
	// 1st 2nd col
	stp		q0, q1, [x8, #(0*16)]
	stp		q4, q5, [x12, #(0*16)]
	// 3rd 4th col
	stp		q2, q3, [x8, #(2*16)]
	stp		q6, q7, [x12, #(2*16)]

	// 5th col
	str		q8, [x8, #(4*16)]
	str		q12, [x12, #(4*16)]
	cmp		w11, #6
	blt		0f
	// 6th col
	str		q9, [x8, #(5*16)]
	str		q13, [x12, #(5*16)]
	cmp		w11, #7
	blt		0f
	// 7th col
	str		q10, [x8, #(6*16)]
	str		q14, [x12, #(6*16)]
	beq		0f
	// 8th col
	str		q11, [x8, #(7*16)]
	str		q15, [x12, #(7*16)]


0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_8x8_vs_lib4)
#endif





//                               w0        x1             x2         w3       x4         w5       x6             x7        sp+0     sp+8       sp+16
// void kernel_sgemm_nt_8x4_lib4(int kmax, double *alpha, double *A, int sda, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd)

	.align	4
	GLOB(kernel_sgemm_nt_8x8_lib4)
	FUN_START(kernel_sgemm_nt_8x8_lib4)
	


	PROLOGUE



	ZERO_ACC



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



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // D
	lsl		w11, w11, #4 // 16*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8_LIB4
#else
	CALL(inner_scale_ab_8x8_lib4)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // sdd
	lsl		w9, w9, #4 // 16*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_8X8_LIB4
#else
	CALL(inner_store_8x8_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_nt_8x8_lib4)





//                                  w0        x1             x2         w3       x4         w5       x6             x7        sp+0     sp+8       sp+16    sp+24       s[+32
// void kernel_sgemm_nt_8x4_vs_lib4(int kmax, double *alpha, double *A, int sda, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd, int m1, int n1)

	.align	4
	GLOB(kernel_sgemm_nt_8x8_vs_lib4)
	FUN_START(kernel_sgemm_nt_8x8_vs_lib4)
	


	PROLOGUE



	ZERO_ACC



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



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // D
	lsl		w11, w11, #4 // 16*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8_LIB4
#else
	CALL(inner_scale_ab_8x8_lib4)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // sdd
	lsl		w9, w9, #4 // 16*sdd
	ldr		w10, [sp, #(STACKSIZE + 24)] // m1
	ldr		w11, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X8_VS_LIB4
#else
	CALL(inner_store_8x8_vs_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_nt_8x8_vs_lib4)





//                               w0        x1             x2         w3       w4           x5         w6       x7            sp+0       sp+8     sp+16      sp+24
// void kernel_sgemm_nn_8x4_lib4(int kmax, double *alpha, double *A, int sda, int offsetB, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd)

	.align	4
	GLOB(kernel_sgemm_nn_8x8_lib4)
	FUN_START(kernel_sgemm_nn_8x8_lib4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #4 // 16*sda
	mov		x11, x5 // B
	mov		w12, w6 // sdb
	lsl		w12, w12, #4 // 16*sdb
	mov		w13, w4 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMM_ADD_NN_8X8_LIB4
#else
	CALL(inner_edge_gemm_add_nn_8x8_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X8_LIB4
#else
	CALL(inner_kernel_gemm_add_nn_8x8_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x7 // beta
	ldr		x10, [sp, #(STACKSIZE + 0)] // C
	ldr		w11, [sp, #(STACKSIZE + 8)] // sdc
	lsl		w11, w11, #4 // 16*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8_LIB4
#else
	CALL(inner_scale_ab_8x8_lib4)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 16)] // D
	ldr		w9, [sp, #(STACKSIZE + 24)] // sdd
	lsl		w9, w9, #4 // 16*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_8X8_LIB4
#else
	CALL(inner_store_8x8_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_nn_8x8_lib4)




//                                  w0        x1             x2         w3       w4           x5         w6       x7            sp+0       sp+8     sp+16      sp+24    sp+32   sp+40
// void kernel_sgemm_nn_8x4_vs_lib4(int kmax, double *alpha, double *A, int sda, int offsetB, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd, int m1, int n1)

	.align	4
	GLOB(kernel_sgemm_nn_8x8_vs_lib4)
	FUN_START(kernel_sgemm_nn_8x8_vs_lib4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #4 // 16*sda
	mov		x11, x5 // B
	mov		w12, w6 // sdb
	lsl		w12, w12, #4 // 16*sdb
	mov		w13, w4 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMM_ADD_NN_8X8_LIB4
#else
	CALL(inner_edge_gemm_add_nn_8x8_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X8_LIB4
#else
	CALL(inner_kernel_gemm_add_nn_8x8_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x7 // beta
	ldr		x10, [sp, #(STACKSIZE + 0)] // C
	ldr		w11, [sp, #(STACKSIZE + 8)] // sdc
	lsl		w11, w11, #4 // 16*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8_LIB4
#else
	CALL(inner_scale_ab_8x8_lib4)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 16)] // D
	ldr		w9, [sp, #(STACKSIZE + 24)] // sdd
	lsl		w9, w9, #4 // 16*sdd
	ldr		w10, [sp, #(STACKSIZE + 32)] // m1
	ldr		w11, [sp, #(STACKSIZE + 40)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X8_VS_LIB4
#else
	CALL(inner_store_8x8_vs_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_nn_8x8_vs_lib4)






//#if defined(BLAS_API)
#if ( defined(BLAS_API) | ( defined(LA_HIGH_PERFORMANCE) & defined(MF_COLMAJ) ) )

#include "kernel_sgemm_8x8_lib.S"

#endif

