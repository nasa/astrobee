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
	fmov    d15, d0; \
	fmov    d16, d0; \
	fmov    d17, d0; \
	fmov    d18, d0; \
	fmov    d19, d0; \
	fmov    d20, d0; \
	fmov    d21, d0; \
	fmov    d22, d0; \
	fmov    d23, d0

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
	fmov    d16, d0
	fmov    d17, d0
	fmov    d18, d0
	fmov    d19, d0
	fmov    d20, d0
	fmov    d21, d0
	fmov    d22, d0
	fmov    d23, d0
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


	// prefetch
	// B
	prfm	PLDL1KEEP, [x11, #0]
	// A 1st panel
	prfm	PLDL1KEEP, [x9, #0]
	// A 2nd panel
	add		x12, x9, x10
	prfm	PLDL1KEEP, [x12, #0]
	// A 3rd panel
	add		x13, x12, x10
	prfm	PLDL1KEEP, [x13, #0]


	// preload
	// 1st loop
	// A
	ldr		d24, [x9, #(0*8+0*32)] // A0[0]
	ldr		x20, [x9, #(1*8+0*32)] // A0[1]
	ldr		d25, [x9, #(2*8+0*32)] // A1[0]
	ldr		x21, [x9, #(3*8+0*32)] // A1[1]
	ldr		d26, [x12, #(0*8+0*32)] // A2[0]
	ldr		x22, [x12, #(1*8+0*32)] // A2[1]
	ldr		d27, [x12, #(2*8+0*32)] // A3[0]
	ldr		x23, [x12, #(3*8+0*32)] // A3[1]
	//                              // A4[0]
	ldr		x24, [x13, #(1*8+0*32)] // A4[1]
	//                              // A5[0]
	ldr		x25, [x13, #(3*8+0*32)] // A5[1]
	// B
	ldr		d30, [x11, #(0*8+0*32)] // B0[0]
	ldr		x26, [x11, #(1*8+0*32)] // B0[1]
	//                              // B1[0]
	ldr		x27, [x11, #(3*8+0*32)] // B1[1]

	prfm	PLDL1KEEP, [x11, #64]
	ins		v24.d[1], x20 // A0[1]
	prfm	PLDL1KEEP, [x9, #64]
	ins		v25.d[1], x21 // A1[1]
	prfm	PLDL1KEEP, [x12, #64]
	ins		v30.d[1], x26 // B0[1]
	prfm	PLDL1KEEP, [x13, #64]

	// 2nd loop
	// A
	ldr		x20, [x9, #(1*8+1*32)] // A0[1]
	ldr		x21, [x9, #(3*8+1*32)] // A1[1]
	// B
	ldr		x26, [x11, #(1*8+1*32)] // B0[1]


	cmp		w8, #4
	ble		0f // consider clean up loop

	// main loop
1:

//	ldr		d30, [x11], #8 // B0[0]
//	ldr		x26, [x11], #8 // B0[1]
//	ins		v30.d[1], x26 // B0[1]
//	ld1		{v30.d}[1], [x11], #8 // B0[1]


	// unroll 0

	// load 31 ins 26
	ldr		d31, [x11, #(2*8+0*32)] // B1[0]
	ins		v26.d[1], x22 // A2[1]
	fmla	v0.2d, v24.2d, v30.d[0]
	ldr		x22, [x12, #(1*8+1*32)] // A2[1]
	fmla	v2.2d, v24.2d, v30.d[1]
	prfm	PLDL1KEEP, [x11, #128]
	fmla	v1.2d, v25.2d, v30.d[0]

	// load 28 ins 31
	ldr		d28, [x13, #(0*8+0*32)] // A4[0]
	ins		v31.d[1], x27 // B1[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	ldr		x27, [x11, #(3*8+1*32)] // B1[1]
	fmla	v8.2d, v26.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, #128]
	fmla	v10.2d, v26.2d, v30.d[1]

	// load 29 ins 27
	ldr		d29, [x13, #(2*8+0*32)] // A5[0]
	ins		v27.d[1], x23 // A3[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	ldr		x23, [x12, #(3*8+1*32)] // A3[1]
	fmla	v6.2d, v24.2d, v31.d[1]
	prfm	PLDL1KEEP, [x12, #128]
	fmla	v5.2d, v25.2d, v31.d[0]

	// load 24 ins 28
	ldr		d24, [x9, #(0*8+1*32)] // A0[0]
	ins		v28.d[1], x24 // A4[1]
	fmla	v7.2d, v25.2d, v31.d[1]
	ldr		x24, [x13, #(1*8+1*32)] // A4[1]
	fmla	v12.2d, v26.2d, v31.d[0]
	prfm	PLDL1KEEP, [x13, #128]
	fmla	v14.2d, v26.2d, v31.d[1]

	// load 25 ins 29
	ldr		d25, [x9, #(2*8+1*32)] // A1[0]
	ins		v29.d[1], x25 // A5[1]
	fmla	v9.2d, v27.2d, v30.d[0]
	ldr		x25, [x13, #(3*8+1*32)] // A5[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v16.2d, v28.2d, v30.d[0]

	// load 26 ins 24
	ldr		d26, [x12, #(0*8+1*32)] // A2[0]
	ins		v24.d[1], x20 // A0[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	ldr		x20, [x9, #(1*8+2*32)] // A0[1]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v19.2d, v29.2d, v30.d[1]

	// load 30 ins 25
	ldr		d30, [x11, #(0*8+1*32)] // B0[0]
	ins		v25.d[1], x21 // A1[1]
	fmla	v13.2d, v27.2d, v31.d[0]
	ldr		x21, [x9, #(3*8+2*32)] // A1[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	fmla	v20.2d, v28.2d, v31.d[0]

	// load 27 ins 30
	ldr		d27, [x12, #(2*8+1*32)] // A3[0]
	ins		v30.d[1], x26 // B0[1]
	fmla	v22.2d, v28.2d, v31.d[1]
	ldr		x26, [x11, #(1*8+2*32)] // B0[1]
	fmla	v21.2d, v29.2d, v31.d[0]
	fmla	v23.2d, v29.2d, v31.d[1]


	// unroll 1

	// load 31 ins 26
	ldr		d31, [x11, #(2*8+1*32)] // B1[0]
	ins		v26.d[1], x22 // A2[1]
	fmla	v0.2d, v24.2d, v30.d[0]
	ldr		x22, [x12, #(1*8+2*32)] // A2[1]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v1.2d, v25.2d, v30.d[0]

	// load 28 ins 31
	ldr		d28, [x13, #(0*8+1*32)] // A4[0]
	ins		v31.d[1], x27 // B1[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	ldr		x27, [x11, #(3*8+2*32)] // B1[1]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v10.2d, v26.2d, v30.d[1]

	// load 29 ins 27
	ldr		d29, [x13, #(2*8+1*32)] // A5[0]
	ins		v27.d[1], x23 // A3[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	ldr		x23, [x12, #(3*8+2*32)] // A3[1]
	fmla	v6.2d, v24.2d, v31.d[1]
	fmla	v5.2d, v25.2d, v31.d[0]

	// load 24 ins 28
	ldr		d24, [x9, #(0*8+2*32)] // A0[0]
	ins		v28.d[1], x24 // A4[1]
	fmla	v7.2d, v25.2d, v31.d[1]
	ldr		x24, [x13, #(1*8+2*32)] // A4[1]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v14.2d, v26.2d, v31.d[1]

	// load 25 ins 29
	ldr		d25, [x9, #(2*8+2*32)] // A1[0]
	ins		v29.d[1], x25 // A5[1]
	fmla	v9.2d, v27.2d, v30.d[0]
	ldr		x25, [x13, #(3*8+2*32)] // A5[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v16.2d, v28.2d, v30.d[0]

	// load 26 ins 24
	ldr		d26, [x12, #(0*8+2*32)] // A2[0]
	ins		v24.d[1], x20 // A0[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	ldr		x20, [x9, #(1*8+3*32)] // A0[1]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v19.2d, v29.2d, v30.d[1]

	// load 30 ins 25
	ldr		d30, [x11, #(0*8+2*32)] // B0[0]
	ins		v25.d[1], x21 // A1[1]
	fmla	v13.2d, v27.2d, v31.d[0]
	ldr		x21, [x9, #(3*8+3*32)] // A1[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	fmla	v20.2d, v28.2d, v31.d[0]

	// load 27 ins 30
	ldr		d27, [x12, #(2*8+2*32)] // A3[0]
	ins		v30.d[1], x26 // B0[1]
	fmla	v22.2d, v28.2d, v31.d[1]
	ldr		x26, [x11, #(1*8+3*32)] // B0[1]
	fmla	v21.2d, v29.2d, v31.d[0]
	fmla	v23.2d, v29.2d, v31.d[1]


	// unroll 2

	// load 31 ins 26
	ldr		d31, [x11, #(2*8+2*32)] // B1[0]
	ins		v26.d[1], x22 // A2[1]
	fmla	v0.2d, v24.2d, v30.d[0]
	ldr		x22, [x12, #(1*8+3*32)] // A2[1]
	fmla	v2.2d, v24.2d, v30.d[1]
	prfm	PLDL1KEEP, [x11, #192]
	fmla	v1.2d, v25.2d, v30.d[0]

	// load 28 ins 31
	ldr		d28, [x13, #(0*8+2*32)] // A4[0]
	ins		v31.d[1], x27 // B1[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	ldr		x27, [x11, #(3*8+3*32)] // B1[1]
	fmla	v8.2d, v26.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, #192]
	fmla	v10.2d, v26.2d, v30.d[1]

	// load 29 ins 27
	ldr		d29, [x13, #(2*8+2*32)] // A5[0]
	ins		v27.d[1], x23 // A3[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	ldr		x23, [x12, #(3*8+3*32)] // A3[1]
	fmla	v6.2d, v24.2d, v31.d[1]
	prfm	PLDL1KEEP, [x12, #192]
	fmla	v5.2d, v25.2d, v31.d[0]

	// load 24 ins 28
	ldr		d24, [x9, #(0*8+3*32)] // A0[0]
	ins		v28.d[1], x24 // A4[1]
	fmla	v7.2d, v25.2d, v31.d[1]
	ldr		x24, [x13, #(1*8+3*32)] // A4[1]
	fmla	v12.2d, v26.2d, v31.d[0]
	prfm	PLDL1KEEP, [x13, #192]
	fmla	v14.2d, v26.2d, v31.d[1]

	// load 25 ins 29
	ldr		d25, [x9, #(2*8+3*32)] // A1[0]
	ins		v29.d[1], x25 // A5[1]
	fmla	v9.2d, v27.2d, v30.d[0]
	ldr		x25, [x13, #(3*8+3*32)] // A5[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v16.2d, v28.2d, v30.d[0]

	// load 26 ins 24
	ldr		d26, [x12, #(0*8+3*32)] // A2[0]
	ins		v24.d[1], x20 // A0[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	ldr		x20, [x9, #(1*8+4*32)] // A0[1]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v19.2d, v29.2d, v30.d[1]

	// load 30 ins 25
	ldr		d30, [x11, #(0*8+3*32)] // B0[0]
	ins		v25.d[1], x21 // A1[1]
	fmla	v13.2d, v27.2d, v31.d[0]
	ldr		x21, [x9, #(3*8+4*32)] // A1[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	fmla	v20.2d, v28.2d, v31.d[0]

	// load 27 ins 30
	ldr		d27, [x12, #(2*8+3*32)] // A3[0]
	ins		v30.d[1], x26 // B0[1]
	fmla	v22.2d, v28.2d, v31.d[1]
	ldr		x26, [x11, #(1*8+4*32)] // B0[1]
	fmla	v21.2d, v29.2d, v31.d[0]
	fmla	v23.2d, v29.2d, v31.d[1]


	// unroll 3

	// load 31 ins 26
	ldr		d31, [x11, #(2*8+3*32)] // B1[0]
	ins		v26.d[1], x22 // A2[1]
	fmla	v0.2d, v24.2d, v30.d[0]
	ldr		x22, [x12, #(1*8+4*32)] // A2[1]
	fmla	v2.2d, v24.2d, v30.d[1]
	add		x9, x9, #(4*32) // A_0
	fmla	v1.2d, v25.2d, v30.d[0]

	// load 28 ins 31
	ldr		d28, [x13, #(0*8+3*32)] // A4[0]
	ins		v31.d[1], x27 // B1[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	ldr		x27, [x11, #(3*8+4*32)] // B1[1]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v10.2d, v26.2d, v30.d[1]

	// load 29 ins 27
	ldr		d29, [x13, #(2*8+3*32)] // A5[0]
	ins		v27.d[1], x23 // A3[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	ldr		x23, [x12, #(3*8+4*32)] // A3[1]
	fmla	v6.2d, v24.2d, v31.d[1]
	sub		w8, w8, #4
	fmla	v5.2d, v25.2d, v31.d[0]

	// load 24 ins 28
	ldr		d24, [x9, #(0*8+0*32)] // A0[0]
	ins		v28.d[1], x24 // A4[1]
	fmla	v7.2d, v25.2d, v31.d[1]
	ldr		x24, [x13, #(1*8+4*32)] // A4[1]
	fmla	v12.2d, v26.2d, v31.d[0]
	add		x12, x12, #(4*32) // A_1
	fmla	v14.2d, v26.2d, v31.d[1]

	// load 25 ins 29
	ldr		d25, [x9, #(2*8+0*32)] // A1[0]
	ins		v29.d[1], x25 // A5[1]
	fmla	v9.2d, v27.2d, v30.d[0]
	ldr		x25, [x13, #(3*8+4*32)] // A5[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	add		x11, x11, #(4*32) // B
	fmla	v16.2d, v28.2d, v30.d[0]

	// load 26 ins 24
	ldr		d26, [x12, #(0*8+0*32)] // A2[0]
	ins		v24.d[1], x20 // A0[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	ldr		x20, [x9, #(1*8+1*32)] // A0[1]
	fmla	v17.2d, v29.2d, v30.d[0]
	add		x13, x13, #(4*32) // A_2
	fmla	v19.2d, v29.2d, v30.d[1]

	// load 30 ins 25
	ldr		d30, [x11, #(0*8+0*32)] // B0[0]
	ins		v25.d[1], x21 // A1[1]
	fmla	v13.2d, v27.2d, v31.d[0]
	ldr		x21, [x9, #(3*8+1*32)] // A1[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	cmp		w8, #4
	fmla	v20.2d, v28.2d, v31.d[0]

	// load 27 ins 30
	ldr		d27, [x12, #(2*8+0*32)] // A3[0]
	ins		v30.d[1], x26 // B0[1]
	fmla	v22.2d, v28.2d, v31.d[1]
	ldr		x26, [x11, #(1*8+1*32)] // B0[1]
	fmla	v21.2d, v29.2d, v31.d[0]
	fmla	v23.2d, v29.2d, v31.d[1]


	bgt		1b

0:

//	b 2f // XXX

	cmp		w8, #3
	ble		4f


	// unroll 0

	// load 31 ins 26
	ldr		d31, [x11, #(2*8+0*32)] // B1[0]
	ins		v26.d[1], x22 // A2[1]
	fmla	v0.2d, v24.2d, v30.d[0]
	ldr		x22, [x12, #(1*8+1*32)] // A2[1]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v1.2d, v25.2d, v30.d[0]

	// load 28 ins 31
	ldr		d28, [x13, #(0*8+0*32)] // A4[0]
	ins		v31.d[1], x27 // B1[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	ldr		x27, [x11, #(3*8+1*32)] // B1[1]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v10.2d, v26.2d, v30.d[1]

	// load 29 ins 27
	ldr		d29, [x13, #(2*8+0*32)] // A5[0]
	ins		v27.d[1], x23 // A3[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	ldr		x23, [x12, #(3*8+1*32)] // A3[1]
	fmla	v6.2d, v24.2d, v31.d[1]
	fmla	v5.2d, v25.2d, v31.d[0]

	// load 24 ins 28
	ldr		d24, [x9, #(0*8+1*32)] // A0[0]
	ins		v28.d[1], x24 // A4[1]
	fmla	v7.2d, v25.2d, v31.d[1]
	ldr		x24, [x13, #(1*8+1*32)] // A4[1]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v14.2d, v26.2d, v31.d[1]

	// load 25 ins 29
	ldr		d25, [x9, #(2*8+1*32)] // A1[0]
	ins		v29.d[1], x25 // A5[1]
	fmla	v9.2d, v27.2d, v30.d[0]
	ldr		x25, [x13, #(3*8+1*32)] // A5[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v16.2d, v28.2d, v30.d[0]

	// load 26 ins 24
	ldr		d26, [x12, #(0*8+1*32)] // A2[0]
	ins		v24.d[1], x20 // A0[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	ldr		x20, [x9, #(1*8+2*32)] // A0[1]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v19.2d, v29.2d, v30.d[1]

	// load 30 ins 25
	ldr		d30, [x11, #(0*8+1*32)] // B0[0]
	ins		v25.d[1], x21 // A1[1]
	fmla	v13.2d, v27.2d, v31.d[0]
	ldr		x21, [x9, #(3*8+2*32)] // A1[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	fmla	v20.2d, v28.2d, v31.d[0]

	// load 27 ins 30
	ldr		d27, [x12, #(2*8+1*32)] // A3[0]
	ins		v30.d[1], x26 // B0[1]
	fmla	v22.2d, v28.2d, v31.d[1]
	ldr		x26, [x11, #(1*8+2*32)] // B0[1]
	fmla	v21.2d, v29.2d, v31.d[0]
	fmla	v23.2d, v29.2d, v31.d[1]


	// unroll 1

	// load 31 ins 26
	ldr		d31, [x11, #(2*8+1*32)] // B1[0]
	ins		v26.d[1], x22 // A2[1]
	fmla	v0.2d, v24.2d, v30.d[0]
	ldr		x22, [x12, #(1*8+2*32)] // A2[1]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v1.2d, v25.2d, v30.d[0]

	// load 28 ins 31
	ldr		d28, [x13, #(0*8+1*32)] // A4[0]
	ins		v31.d[1], x27 // B1[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	ldr		x27, [x11, #(3*8+2*32)] // B1[1]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v10.2d, v26.2d, v30.d[1]

	// load 29 ins 27
	ldr		d29, [x13, #(2*8+1*32)] // A5[0]
	ins		v27.d[1], x23 // A3[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	ldr		x23, [x12, #(3*8+2*32)] // A3[1]
	fmla	v6.2d, v24.2d, v31.d[1]
	fmla	v5.2d, v25.2d, v31.d[0]

	// load 24 ins 28
	ldr		d24, [x9, #(0*8+2*32)] // A0[0]
	ins		v28.d[1], x24 // A4[1]
	fmla	v7.2d, v25.2d, v31.d[1]
	ldr		x24, [x13, #(1*8+2*32)] // A4[1]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v14.2d, v26.2d, v31.d[1]

	// load 25 ins 29
	ldr		d25, [x9, #(2*8+2*32)] // A1[0]
	ins		v29.d[1], x25 // A5[1]
	fmla	v9.2d, v27.2d, v30.d[0]
	ldr		x25, [x13, #(3*8+2*32)] // A5[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v16.2d, v28.2d, v30.d[0]

	// load 26 ins 24
	ldr		d26, [x12, #(0*8+2*32)] // A2[0]
	ins		v24.d[1], x20 // A0[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	ldr		x20, [x9, #(1*8+3*32)] // A0[1]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v19.2d, v29.2d, v30.d[1]

	// load 30 ins 25
	ldr		d30, [x11, #(0*8+2*32)] // B0[0]
	ins		v25.d[1], x21 // A1[1]
	fmla	v13.2d, v27.2d, v31.d[0]
	ldr		x21, [x9, #(3*8+3*32)] // A1[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	fmla	v20.2d, v28.2d, v31.d[0]

	// load 27 ins 30
	ldr		d27, [x12, #(2*8+2*32)] // A3[0]
	ins		v30.d[1], x26 // B0[1]
	fmla	v22.2d, v28.2d, v31.d[1]
	ldr		x26, [x11, #(1*8+3*32)] // B0[1]
	fmla	v21.2d, v29.2d, v31.d[0]
	fmla	v23.2d, v29.2d, v31.d[1]


	// unroll 2

	// load 31 ins 26
	ldr		d31, [x11, #(2*8+2*32)] // B1[0]
	ins		v26.d[1], x22 // A2[1]
	fmla	v0.2d, v24.2d, v30.d[0]
	ldr		x22, [x12, #(1*8+3*32)] // A2[1]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v1.2d, v25.2d, v30.d[0]

	// load 28 ins 31
	ldr		d28, [x13, #(0*8+2*32)] // A4[0]
	ins		v31.d[1], x27 // B1[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	ldr		x27, [x11, #(3*8+3*32)] // B1[1]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v10.2d, v26.2d, v30.d[1]

	// load 29 ins 27
	ldr		d29, [x13, #(2*8+2*32)] // A5[0]
	ins		v27.d[1], x23 // A3[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	ldr		x23, [x12, #(3*8+3*32)] // A3[1]
	fmla	v6.2d, v24.2d, v31.d[1]
	fmla	v5.2d, v25.2d, v31.d[0]

	// load 24 ins 28
	ldr		d24, [x9, #(0*8+3*32)] // A0[0]
	ins		v28.d[1], x24 // A4[1]
	fmla	v7.2d, v25.2d, v31.d[1]
	ldr		x24, [x13, #(1*8+3*32)] // A4[1]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v14.2d, v26.2d, v31.d[1]

	// load 25 ins 29
	ldr		d25, [x9, #(2*8+3*32)] // A1[0]
	ins		v29.d[1], x25 // A5[1]
	fmla	v9.2d, v27.2d, v30.d[0]
	ldr		x25, [x13, #(3*8+3*32)] // A5[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v16.2d, v28.2d, v30.d[0]

	// load 26 ins 24
	ldr		d26, [x12, #(0*8+3*32)] // A2[0]
	ins		v24.d[1], x20 // A0[1]
	fmla	v18.2d, v28.2d, v30.d[1]
//	ldr		x20, [x9, #(1*8+4*32)] // A0[1]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v19.2d, v29.2d, v30.d[1]

	// load 30 ins 25
	ldr		d30, [x11, #(0*8+3*32)] // B0[0]
	ins		v25.d[1], x21 // A1[1]
	fmla	v13.2d, v27.2d, v31.d[0]
//	ldr		x21, [x9, #(3*8+4*32)] // A1[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	fmla	v20.2d, v28.2d, v31.d[0]

	// load 27 ins 30
	ldr		d27, [x12, #(2*8+3*32)] // A3[0]
	ins		v30.d[1], x26 // B0[1]
	fmla	v22.2d, v28.2d, v31.d[1]
//	ldr		x26, [x11, #(1*8+4*32)] // B0[1]
	fmla	v21.2d, v29.2d, v31.d[0]
	fmla	v23.2d, v29.2d, v31.d[1]


	// unroll 3

	// load 31 ins 26
	ldr		d31, [x11, #(2*8+3*32)] // B1[0]
	ins		v26.d[1], x22 // A2[1]
	fmla	v0.2d, v24.2d, v30.d[0]
//	ldr		x22, [x12, #(1*8+4*32)] // A2[1]
	fmla	v2.2d, v24.2d, v30.d[1]
	add		x9, x9, #(4*32) // A_0
	fmla	v1.2d, v25.2d, v30.d[0]

	// load 28 ins 31
	ldr		d28, [x13, #(0*8+3*32)] // A4[0]
	ins		v31.d[1], x27 // B1[1]
	fmla	v3.2d, v25.2d, v30.d[1]
//	ldr		x27, [x11, #(3*8+4*32)] // B1[1]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v10.2d, v26.2d, v30.d[1]

	// load 29 ins 27
	ldr		d29, [x13, #(2*8+3*32)] // A5[0]
	ins		v27.d[1], x23 // A3[1]
	fmla	v4.2d, v24.2d, v31.d[0]
//	ldr		x23, [x12, #(3*8+4*32)] // A3[1]
	fmla	v6.2d, v24.2d, v31.d[1]
	sub		w8, w8, #4
	fmla	v5.2d, v25.2d, v31.d[0]

	// load 24 ins 28
//	ldr		d24, [x9, #(0*8+0*32)] // A0[0]
	ins		v28.d[1], x24 // A4[1]
	fmla	v7.2d, v25.2d, v31.d[1]
//	ldr		x24, [x13, #(1*8+4*32)] // A4[1]
	fmla	v12.2d, v26.2d, v31.d[0]
	add		x12, x12, #(4*32) // A_1
	fmla	v14.2d, v26.2d, v31.d[1]

	// load 25 ins 29
//	ldr		d25, [x9, #(2*8+0*32)] // A1[0]
	ins		v29.d[1], x25 // A5[1]
	fmla	v9.2d, v27.2d, v30.d[0]
//	ldr		x25, [x13, #(3*8+4*32)] // A5[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	add		x11, x11, #(4*32) // B
	fmla	v16.2d, v28.2d, v30.d[0]

	// load 26 ins 24
//	ldr		d26, [x12, #(0*8+0*32)] // A2[0]
//	ins		v24.d[1], x20 // A0[1]
	fmla	v18.2d, v28.2d, v30.d[1]
//	ldr		x20, [x9, #(1*8+1*32)] // A0[1]
	fmla	v17.2d, v29.2d, v30.d[0]
	add		x13, x13, #(4*32) // A_2
	fmla	v19.2d, v29.2d, v30.d[1]

	// load 30 ins 25
//	ldr		d30, [x11, #(0*8+0*32)] // B0[0]
//	ins		v25.d[1], x21 // A1[1]
	fmla	v13.2d, v27.2d, v31.d[0]
//	ldr		x21, [x9, #(3*8+1*32)] // A1[1]
	fmla	v15.2d, v27.2d, v31.d[1]
//	cmp		w8, #4
	fmla	v20.2d, v28.2d, v31.d[0]

	// load 27 ins 30
//	ldr		d27, [x12, #(2*8+0*32)] // A3[0]
//	ins		v30.d[1], x26 // B0[1]
	fmla	v22.2d, v28.2d, v31.d[1]
//	ldr		x26, [x11, #(1*8+1*32)] // B0[1]
	fmla	v21.2d, v29.2d, v31.d[0]
	fmla	v23.2d, v29.2d, v31.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x9, x9, #32
//	sub		x11, x11, #32
//	sub		x12, x12, #32

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16 // A0
	ldr		q25, [x9], #16 // A1
	ldr		q30, [x11], #16 // B0
	ldr		q31, [x11], #16 // B1

	ldr		d26, [x12], #8 // A2[0]
	fmla	v0.2d, v24.2d, v30.d[0]
	ldr		x22, [x12], #8 // A2[1]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v4.2d, v24.2d, v31.d[0]

	ldr		d27, [x12], #8 // A3[0]
	ins		v26.d[1], x22 // A2[1]
	fmla	v6.2d, v24.2d, v31.d[1]
	ldr		x23, [x12], #8 // A3[1]
	fmla	v1.2d, v25.2d, v30.d[0]
	fmla	v3.2d, v25.2d, v30.d[1]

	ldr		d28, [x13], #8 // A4[0]
	ins		v27.d[1], x23 // A3[1]
	fmla	v5.2d, v25.2d, v31.d[0]
	ldr		x24, [x13], #8 // A4[1]
	fmla	v7.2d, v25.2d, v31.d[1]
	fmla	v8.2d, v26.2d, v30.d[0]

	ldr		d29, [x13], #8 // A5[0]
	ins		v28.d[1], x24 // A4[1]
	fmla	v10.2d, v26.2d, v30.d[1]
	ldr		x25, [x13], #8 // A5[1]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]

	ins		v29.d[1], x25 // A5[1]
	fmla	v14.2d, v26.2d, v31.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v13.2d, v27.2d, v31.d[0]

	fmla	v15.2d, v27.2d, v31.d[1]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v18.2d, v28.2d, v30.d[1]

	fmla	v20.2d, v28.2d, v31.d[0]
	sub		w8, w8, #1
	fmla	v22.2d, v28.2d, v31.d[1]
	fmla	v17.2d, v29.2d, v30.d[0]

	fmla	v19.2d, v29.2d, v30.d[1]
	cmp		w8, #0
	fmla	v21.2d, v29.2d, v31.d[0]
	fmla	v23.2d, v29.2d, v31.d[1]

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x9, x10
	add		x13, x12, x10

	// prefetch
	prfm	PLDL1KEEP, [x9, #(0*128+0)]
	prfm	PLDL1KEEP, [x12, #(0*128+0)]
	prfm	PLDL1KEEP, [x13, #(0*128+0)]
	prfm	PLDL1KEEP, [x9, #(0*128+64)]
	prfm	PLDL1KEEP, [x12, #(0*128+64)]
	prfm	PLDL1KEEP, [x13, #(0*128+64)]
	prfm	PLDL1KEEP, [x11, #(0*128+0)]
	prfm	PLDL1KEEP, [x11, #(0*128+64)]

	// preload
	ldp		q24, q25, [x9, #(0*8+0*32)]
	ldp		q26, q27, [x12, #(0*8+0*32)]
	ldp		q28, q29, [x13, #(0*8+0*32)]
	ldp		q30, q31, [x11, #(0*8+0*32)]

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// main loop
1:
	
	// unroll 0
	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, #(1*128+0)]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	prfm	PLDL1KEEP, [x12, #(1*128+0)]
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	prfm	PLDL1KEEP, [x13, #(1*128+0)]

	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	prfm	PLDL1KEEP, [x9, #(1*128+64)]
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	prfm	PLDL1KEEP, [x12, #(1*128+64)]
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]
	prfm	PLDL1KEEP, [x13, #(1*128+64)]

	fmla	v4.2d, v24.2d, v31.d[0]
	fmla	v5.2d, v25.2d, v31.d[0]
	prfm	PLDL1KEEP, [x11, #(1*128+0)]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v13.2d, v27.2d, v31.d[0]
	prfm	PLDL1KEEP, [x11, #(1*128+64)]
	fmla	v20.2d, v28.2d, v31.d[0]
	fmla	v21.2d, v29.2d, v31.d[0]

	fmla	v6.2d, v24.2d, v31.d[1]
	fmla	v7.2d, v25.2d, v31.d[1]
	ldp		q24, q25, [x9, #(0*8+1*32)]
	fmla	v14.2d, v26.2d, v31.d[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	ldp		q26, q27, [x12, #(0*8+1*32)]
	fmla	v22.2d, v28.2d, v31.d[1]
	fmla	v23.2d, v29.2d, v31.d[1]
	ldp		q28, q29, [x13, #(0*8+1*32)]
	ldp		q30, q31, [x11, #(0*8+1*32)]

	// unroll 1
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
	ldp		q24, q25, [x9, #(0*8+2*32)]
	fmla	v14.2d, v26.2d, v31.d[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	ldp		q26, q27, [x12, #(0*8+2*32)]
	fmla	v22.2d, v28.2d, v31.d[1]
	fmla	v23.2d, v29.2d, v31.d[1]
	ldp		q28, q29, [x13, #(0*8+2*32)]
	ldp		q30, q31, [x11, #(0*8+2*32)]

	// unroll 2
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
	ldp		q24, q25, [x9, #(0*8+3*32)]
	fmla	v14.2d, v26.2d, v31.d[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	ldp		q26, q27, [x12, #(0*8+3*32)]
	fmla	v22.2d, v28.2d, v31.d[1]
	fmla	v23.2d, v29.2d, v31.d[1]
	ldp		q28, q29, [x13, #(0*8+3*32)]
	ldp		q30, q31, [x11, #(0*8+3*32)]

	// unroll 3
	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	add		x9, x9, #128
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	add		x12, x12, #128
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	add		x13, x13, #128

	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	add		x11, x11, #128
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	sub		w8, w8, #4
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]
	cmp		w8, #4

	fmla	v4.2d, v24.2d, v31.d[0]
	fmla	v5.2d, v25.2d, v31.d[0]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v13.2d, v27.2d, v31.d[0]
	fmla	v20.2d, v28.2d, v31.d[0]
	fmla	v21.2d, v29.2d, v31.d[0]

	fmla	v6.2d, v24.2d, v31.d[1]
	fmla	v7.2d, v25.2d, v31.d[1]
	ldp		q24, q25, [x9, #(0*8+0*32)]
	fmla	v14.2d, v26.2d, v31.d[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	ldp		q26, q27, [x12, #(0*8+0*32)]
	fmla	v22.2d, v28.2d, v31.d[1]
	fmla	v23.2d, v29.2d, v31.d[1]
	ldp		q28, q29, [x13, #(0*8+0*32)]
	ldp		q30, q31, [x11, #(0*8+0*32)]

	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	
	// unroll 0
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
	ldp		q24, q25, [x9, #(0*8+1*32)]
	fmla	v14.2d, v26.2d, v31.d[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	ldp		q26, q27, [x12, #(0*8+1*32)]
	fmla	v22.2d, v28.2d, v31.d[1]
	fmla	v23.2d, v29.2d, v31.d[1]
	ldp		q28, q29, [x13, #(0*8+1*32)]
	ldp		q30, q31, [x11, #(0*8+1*32)]

	// unroll 1
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
	ldp		q24, q25, [x9, #(0*8+2*32)]
	fmla	v14.2d, v26.2d, v31.d[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	ldp		q26, q27, [x12, #(0*8+2*32)]
	fmla	v22.2d, v28.2d, v31.d[1]
	fmla	v23.2d, v29.2d, v31.d[1]
	ldp		q28, q29, [x13, #(0*8+2*32)]
	ldp		q30, q31, [x11, #(0*8+2*32)]

	// unroll 2
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
	ldp		q24, q25, [x9, #(0*8+3*32)]
	fmla	v14.2d, v26.2d, v31.d[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	ldp		q26, q27, [x12, #(0*8+3*32)]
	fmla	v22.2d, v28.2d, v31.d[1]
	fmla	v23.2d, v29.2d, v31.d[1]
	ldp		q28, q29, [x13, #(0*8+3*32)]
	ldp		q30, q31, [x11, #(0*8+3*32)]

	// unroll 3
	fmla	v0.2d, v24.2d, v30.d[0]
	fmla	v1.2d, v25.2d, v30.d[0]
	add		x9, x9, #128
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v9.2d, v27.2d, v30.d[0]
	add		x12, x12, #128
	fmla	v16.2d, v28.2d, v30.d[0]
	fmla	v17.2d, v29.2d, v30.d[0]
	add		x13, x13, #128

	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	add		x11, x11, #128
	fmla	v10.2d, v26.2d, v30.d[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	sub		w8, w8, #4
	fmla	v18.2d, v28.2d, v30.d[1]
	fmla	v19.2d, v29.2d, v30.d[1]
	cmp		w8, #4

	fmla	v4.2d, v24.2d, v31.d[0]
	fmla	v5.2d, v25.2d, v31.d[0]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v13.2d, v27.2d, v31.d[0]
	fmla	v20.2d, v28.2d, v31.d[0]
	fmla	v21.2d, v29.2d, v31.d[0]

	fmla	v6.2d, v24.2d, v31.d[1]
	fmla	v7.2d, v25.2d, v31.d[1]
//	ldp		q24, q25, [x9, #(0*8+0*32)]
	fmla	v14.2d, v26.2d, v31.d[1]
	fmla	v15.2d, v27.2d, v31.d[1]
//	ldp		q26, q27, [x12, #(0*8+0*32)]
	fmla	v22.2d, v28.2d, v31.d[1]
	fmla	v23.2d, v29.2d, v31.d[1]
//	ldp		q28, q29, [x13, #(0*8+0*32)]
//	ldp		q30, q31, [x11, #(0*8+0*32)]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x9, x9, #32
//	sub		x11, x11, #32
//	sub		x12, x12, #32

3: // clean1-up loop

	// unroll 0
	ld1		{v24.2d, v25.2d}, [x9], #32
	ld1		{v26.2d, v27.2d}, [x12], #32
	ld1		{v28.2d, v29.2d}, [x13], #32
	ld1		{v30.2d, v31.2d}, [x11], #32

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



#endif



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_12x4_lib4)
#endif
// end




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
	.macro INNER_KERNEL_GEMM_ADD_NN_12X4_LIB4
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_12x4_lib4)
#endif

	// early return
	cmp		w8, #0
	ble		2f // return


	// prefetch
	// B
	prfm	PLDL1KEEP, [x11, #0]
	// A 1st panel
	prfm	PLDL1KEEP, [x9, #0]
	// A 2nd panel
	add		x13, x9, x10
	prfm	PLDL1KEEP, [x13, #0]
	// A 3rd panel
	add		x14, x13, x10
	prfm	PLDL1KEEP, [x14, #0]


	// preload
	// 1st loop
	// A
	ldr		d24, [x9, #(0*8+0*32)] // A0[0]
	ldr		x20, [x9, #(1*8+0*32)] // A0[1]
	ldr		d25, [x9, #(2*8+0*32)] // A1[0]
	ldr		x21, [x9, #(3*8+0*32)] // A1[1]
	ldr		d26, [x13, #(0*8+0*32)] // A2[0]
	ldr		x22, [x13, #(1*8+0*32)] // A2[1]
	ldr		d27, [x13, #(2*8+0*32)] // A3[0]
	ldr		x23, [x13, #(3*8+0*32)] // A3[1]
	//                              // A4[0]
	ldr		x24, [x14, #(1*8+0*32)] // A4[1]
	//                              // A5[0]
	ldr		x25, [x14, #(3*8+0*32)] // A5[1]
	// B
	ldr		d30, [x11, #(0*8+0*32)] // B0[0]
	ldr		x26, [x11, #(0*8+1*32)] // B0[1]
	//                              // B1[0]
	ldr		x27, [x11, #(0*8+3*32)] // B1[1]

	prfm	PLDL1KEEP, [x11, #64]
	ins		v24.d[1], x20 // A0[1]
	prfm	PLDL1KEEP, [x9, #64]
	ins		v25.d[1], x21 // A1[1]
	prfm	PLDL1KEEP, [x13, #64]
	ins		v30.d[1], x26 // B0[1]
	prfm	PLDL1KEEP, [x14, #64]

	// 2nd loop
	// A
	ldr		x20, [x9, #(1*8+1*32)] // A0[1]
	ldr		x21, [x9, #(3*8+1*32)] // A1[1]
	// B
	ldr		x26, [x11, #(1*8+1*32)] // B0[1]


	cmp		w8, #4
	ble		0f // consider clean up loop

	add		x28, x11, x12

	// main loop
1:

//	ldr		d30, [x11], #8 // B0[0]
//	ldr		x26, [x11], #8 // B0[1]
//	ins		v30.d[1], x26 // B0[1]
//	ld1		{v30.d}[1], [x11], #8 // B0[1]


	// unroll 0

	// load 31 ins 26
	ldr		d31, [x11, #(0*8+2*32)] // B1[0]
	ins		v26.d[1], x22 // A2[1]
	fmla	v0.2d, v24.2d, v30.d[0]
	ldr		x22, [x13, #(1*8+1*32)] // A2[1]
	fmla	v2.2d, v24.2d, v30.d[1]
	prfm	PLDL1KEEP, [x28]
	fmla	v1.2d, v25.2d, v30.d[0]

	// load 28 ins 31
	ldr		d28, [x14, #(0*8+0*32)] // A4[0]
	ins		v31.d[1], x27 // B1[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	ldr		x27, [x11, #(1*8+3*32)] // B1[1]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v10.2d, v26.2d, v30.d[1]

	// load 29 ins 27
	ldr		d29, [x14, #(2*8+0*32)] // A5[0]
	ins		v27.d[1], x23 // A3[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	ldr		x23, [x13, #(3*8+1*32)] // A3[1]
	fmla	v6.2d, v24.2d, v31.d[1]
	prfm	PLDL1KEEP, [x28, #64]
	fmla	v5.2d, v25.2d, v31.d[0]

	// load 24 ins 28
	ldr		d24, [x9, #(0*8+1*32)] // A0[0]
	ins		v28.d[1], x24 // A4[1]
	fmla	v7.2d, v25.2d, v31.d[1]
	ldr		x24, [x14, #(1*8+1*32)] // A4[1]
	fmla	v12.2d, v26.2d, v31.d[0]
	prfm	PLDL1KEEP, [x9, #128]
	fmla	v14.2d, v26.2d, v31.d[1]

	// load 25 ins 29
	ldr		d25, [x9, #(2*8+1*32)] // A1[0]
	ins		v29.d[1], x25 // A5[1]
	fmla	v9.2d, v27.2d, v30.d[0]
	ldr		x25, [x14, #(3*8+1*32)] // A5[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	prfm	PLDL1KEEP, [x13, #128]
	fmla	v16.2d, v28.2d, v30.d[0]

	// load 26 ins 24
	ldr		d26, [x13, #(0*8+1*32)] // A2[0]
	ins		v24.d[1], x20 // A0[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	ldr		x20, [x9, #(1*8+2*32)] // A0[1]
	fmla	v17.2d, v29.2d, v30.d[0]
	prfm	PLDL1KEEP, [x14, #128]
	fmla	v19.2d, v29.2d, v30.d[1]

	// load 30 ins 25
	ldr		d30, [x11, #(1*8+0*32)] // B0[0]
	ins		v25.d[1], x21 // A1[1]
	fmla	v13.2d, v27.2d, v31.d[0]
	ldr		x21, [x9, #(3*8+2*32)] // A1[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	fmla	v20.2d, v28.2d, v31.d[0]

	// load 27 ins 30
	ldr		d27, [x13, #(2*8+1*32)] // A3[0]
	ins		v30.d[1], x26 // B0[1]
	fmla	v22.2d, v28.2d, v31.d[1]
	ldr		x26, [x11, #(2*8+1*32)] // B0[1]
	fmla	v21.2d, v29.2d, v31.d[0]
	fmla	v23.2d, v29.2d, v31.d[1]


	// unroll 1

	// load 31 ins 26
	ldr		d31, [x11, #(1*8+2*32)] // B1[0]
	ins		v26.d[1], x22 // A2[1]
	fmla	v0.2d, v24.2d, v30.d[0]
	ldr		x22, [x13, #(1*8+2*32)] // A2[1]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v1.2d, v25.2d, v30.d[0]

	// load 28 ins 31
	ldr		d28, [x14, #(0*8+1*32)] // A4[0]
	ins		v31.d[1], x27 // B1[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	ldr		x27, [x11, #(2*8+3*32)] // B1[1]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v10.2d, v26.2d, v30.d[1]

	// load 29 ins 27
	ldr		d29, [x14, #(2*8+1*32)] // A5[0]
	ins		v27.d[1], x23 // A3[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	ldr		x23, [x13, #(3*8+2*32)] // A3[1]
	fmla	v6.2d, v24.2d, v31.d[1]
	fmla	v5.2d, v25.2d, v31.d[0]

	// load 24 ins 28
	ldr		d24, [x9, #(0*8+2*32)] // A0[0]
	ins		v28.d[1], x24 // A4[1]
	fmla	v7.2d, v25.2d, v31.d[1]
	ldr		x24, [x14, #(1*8+2*32)] // A4[1]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v14.2d, v26.2d, v31.d[1]

	// load 25 ins 29
	ldr		d25, [x9, #(2*8+2*32)] // A1[0]
	ins		v29.d[1], x25 // A5[1]
	fmla	v9.2d, v27.2d, v30.d[0]
	ldr		x25, [x14, #(3*8+2*32)] // A5[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v16.2d, v28.2d, v30.d[0]

	// load 26 ins 24
	ldr		d26, [x13, #(0*8+2*32)] // A2[0]
	ins		v24.d[1], x20 // A0[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	ldr		x20, [x9, #(1*8+3*32)] // A0[1]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v19.2d, v29.2d, v30.d[1]

	// load 30 ins 25
	ldr		d30, [x11, #(2*8+0*32)] // B0[0]
	ins		v25.d[1], x21 // A1[1]
	fmla	v13.2d, v27.2d, v31.d[0]
	ldr		x21, [x9, #(3*8+3*32)] // A1[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	fmla	v20.2d, v28.2d, v31.d[0]

	// load 27 ins 30
	ldr		d27, [x13, #(2*8+2*32)] // A3[0]
	ins		v30.d[1], x26 // B0[1]
	fmla	v22.2d, v28.2d, v31.d[1]
	ldr		x26, [x11, #(3*8+1*32)] // B0[1]
	fmla	v21.2d, v29.2d, v31.d[0]
	fmla	v23.2d, v29.2d, v31.d[1]


	// unroll 2

	// load 31 ins 26
	ldr		d31, [x11, #(2*8+2*32)] // B1[0]
	ins		v26.d[1], x22 // A2[1]
	fmla	v0.2d, v24.2d, v30.d[0]
	ldr		x22, [x13, #(1*8+3*32)] // A2[1]
	fmla	v2.2d, v24.2d, v30.d[1]
	prfm	PLDL1KEEP, [x9, #192]
	fmla	v1.2d, v25.2d, v30.d[0]

	// load 28 ins 31
	ldr		d28, [x14, #(0*8+2*32)] // A4[0]
	ins		v31.d[1], x27 // B1[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	ldr		x27, [x11, #(3*8+3*32)] // B1[1]
	fmla	v8.2d, v26.2d, v30.d[0]
	prfm	PLDL1KEEP, [x13, #192]
	fmla	v10.2d, v26.2d, v30.d[1]

	// load 29 ins 27
	ldr		d29, [x14, #(2*8+2*32)] // A5[0]
	ins		v27.d[1], x23 // A3[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	ldr		x23, [x13, #(3*8+3*32)] // A3[1]
	fmla	v6.2d, v24.2d, v31.d[1]
	prfm	PLDL1KEEP, [x14, #192]
	fmla	v5.2d, v25.2d, v31.d[0]

	// load 24 ins 28
	ldr		d24, [x9, #(0*8+3*32)] // A0[0]
	ins		v28.d[1], x24 // A4[1]
	fmla	v7.2d, v25.2d, v31.d[1]
	ldr		x24, [x14, #(1*8+3*32)] // A4[1]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v14.2d, v26.2d, v31.d[1]

	// load 25 ins 29
	ldr		d25, [x9, #(2*8+3*32)] // A1[0]
	ins		v29.d[1], x25 // A5[1]
	fmla	v9.2d, v27.2d, v30.d[0]
	ldr		x25, [x14, #(3*8+3*32)] // A5[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v16.2d, v28.2d, v30.d[0]

	// load 26 ins 24
	ldr		d26, [x13, #(0*8+3*32)] // A2[0]
	ins		v24.d[1], x20 // A0[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	ldr		x20, [x9, #(1*8+4*32)] // A0[1]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v19.2d, v29.2d, v30.d[1]

	// load 30 ins 25
	ldr		d30, [x11, #(3*8+0*32)] // B0[0]
	ins		v25.d[1], x21 // A1[1]
	fmla	v13.2d, v27.2d, v31.d[0]
	ldr		x21, [x9, #(3*8+4*32)] // A1[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	fmla	v20.2d, v28.2d, v31.d[0]

	// load 27 ins 30
	ldr		d27, [x13, #(2*8+3*32)] // A3[0]
	ins		v30.d[1], x26 // B0[1]
	fmla	v22.2d, v28.2d, v31.d[1]
	ldr		x26, [x28, #(0*8+1*32)] // B0[1]
	fmla	v21.2d, v29.2d, v31.d[0]
	fmla	v23.2d, v29.2d, v31.d[1]


	// unroll 3

	// load 31 ins 26
	ldr		d31, [x11, #(3*8+2*32)] // B1[0]
	ins		v26.d[1], x22 // A2[1]
	fmla	v0.2d, v24.2d, v30.d[0]
	ldr		x22, [x13, #(1*8+4*32)] // A2[1]
	fmla	v2.2d, v24.2d, v30.d[1]
	add		x9, x9, #(4*32) // A_0
	fmla	v1.2d, v25.2d, v30.d[0]

	// load 28 ins 31
	ldr		d28, [x14, #(0*8+3*32)] // A4[0]
	ins		v31.d[1], x27 // B1[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	ldr		x27, [x28, #(0*8+3*32)] // B1[1]
	fmla	v8.2d, v26.2d, v30.d[0]
	sub		w8, w8, #4
	fmla	v10.2d, v26.2d, v30.d[1]

	// load 29 ins 27
	ldr		d29, [x14, #(2*8+3*32)] // A5[0]
	ins		v27.d[1], x23 // A3[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	ldr		x23, [x13, #(3*8+4*32)] // A3[1]
	fmla	v6.2d, v24.2d, v31.d[1]
	add		x13, x13, #(4*32) // A_1
	fmla	v5.2d, v25.2d, v31.d[0]

	// load 24 ins 28
	ldr		d24, [x9, #(0*8+0*32)] // A0[0]
	ins		v28.d[1], x24 // A4[1]
	fmla	v7.2d, v25.2d, v31.d[1]
	ldr		x24, [x14, #(1*8+4*32)] // A4[1]
	fmla	v12.2d, v26.2d, v31.d[0]
	add		x11, x11, x12 // B
	fmla	v14.2d, v26.2d, v31.d[1]

	// load 25 ins 29
	ldr		d25, [x9, #(2*8+0*32)] // A1[0]
	ins		v29.d[1], x25 // A5[1]
	fmla	v9.2d, v27.2d, v30.d[0]
	ldr		x25, [x14, #(3*8+4*32)] // A5[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	add		x28, x11, x12
	fmla	v16.2d, v28.2d, v30.d[0]

	// load 26 ins 24
	ldr		d26, [x13, #(0*8+0*32)] // A2[0]
	ins		v24.d[1], x20 // A0[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	ldr		x20, [x9, #(1*8+1*32)] // A0[1]
	fmla	v17.2d, v29.2d, v30.d[0]
	add		x14, x14, #(4*32) // A_2
	fmla	v19.2d, v29.2d, v30.d[1]

	// load 30 ins 25
	ldr		d30, [x11, #(0*8+0*32)] // B0[0]
	ins		v25.d[1], x21 // A1[1]
	fmla	v13.2d, v27.2d, v31.d[0]
	ldr		x21, [x9, #(3*8+1*32)] // A1[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	cmp		w8, #4
	fmla	v20.2d, v28.2d, v31.d[0]

	// load 27 ins 30
	ldr		d27, [x13, #(2*8+0*32)] // A3[0]
	ins		v30.d[1], x26 // B0[1]
	fmla	v22.2d, v28.2d, v31.d[1]
	ldr		x26, [x11, #(1*8+1*32)] // B0[1]
	fmla	v21.2d, v29.2d, v31.d[0]
	fmla	v23.2d, v29.2d, v31.d[1]


	bgt		1b

0:

//	b 2f // XXX

	cmp		w8, #3
	ble		4f


	// unroll 0

	// load 31 ins 26
	ldr		d31, [x11, #(0*8+2*32)] // B1[0]
	ins		v26.d[1], x22 // A2[1]
	fmla	v0.2d, v24.2d, v30.d[0]
	ldr		x22, [x13, #(1*8+1*32)] // A2[1]
	fmla	v2.2d, v24.2d, v30.d[1]
	add		x28, x11, x12 // B
	fmla	v1.2d, v25.2d, v30.d[0]

	// load 28 ins 31
	ldr		d28, [x14, #(0*8+0*32)] // A4[0]
	ins		v31.d[1], x27 // B1[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	ldr		x27, [x11, #(1*8+3*32)] // B1[1]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v10.2d, v26.2d, v30.d[1]

	// load 29 ins 27
	ldr		d29, [x14, #(2*8+0*32)] // A5[0]
	ins		v27.d[1], x23 // A3[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	ldr		x23, [x13, #(3*8+1*32)] // A3[1]
	fmla	v6.2d, v24.2d, v31.d[1]
	fmla	v5.2d, v25.2d, v31.d[0]

	// load 24 ins 28
	ldr		d24, [x9, #(0*8+1*32)] // A0[0]
	ins		v28.d[1], x24 // A4[1]
	fmla	v7.2d, v25.2d, v31.d[1]
	ldr		x24, [x14, #(1*8+1*32)] // A4[1]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v14.2d, v26.2d, v31.d[1]

	// load 25 ins 29
	ldr		d25, [x9, #(2*8+1*32)] // A1[0]
	ins		v29.d[1], x25 // A5[1]
	fmla	v9.2d, v27.2d, v30.d[0]
	ldr		x25, [x14, #(3*8+1*32)] // A5[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v16.2d, v28.2d, v30.d[0]

	// load 26 ins 24
	ldr		d26, [x13, #(0*8+1*32)] // A2[0]
	ins		v24.d[1], x20 // A0[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	ldr		x20, [x9, #(1*8+2*32)] // A0[1]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v19.2d, v29.2d, v30.d[1]

	// load 30 ins 25
	ldr		d30, [x11, #(1*8+0*32)] // B0[0]
	ins		v25.d[1], x21 // A1[1]
	fmla	v13.2d, v27.2d, v31.d[0]
	ldr		x21, [x9, #(3*8+2*32)] // A1[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	fmla	v20.2d, v28.2d, v31.d[0]

	// load 27 ins 30
	ldr		d27, [x13, #(2*8+1*32)] // A3[0]
	ins		v30.d[1], x26 // B0[1]
	fmla	v22.2d, v28.2d, v31.d[1]
	ldr		x26, [x11, #(2*8+1*32)] // B0[1]
	fmla	v21.2d, v29.2d, v31.d[0]
	fmla	v23.2d, v29.2d, v31.d[1]


	// unroll 1

	// load 31 ins 26
	ldr		d31, [x11, #(1*8+2*32)] // B1[0]
	ins		v26.d[1], x22 // A2[1]
	fmla	v0.2d, v24.2d, v30.d[0]
	ldr		x22, [x13, #(1*8+2*32)] // A2[1]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v1.2d, v25.2d, v30.d[0]

	// load 28 ins 31
	ldr		d28, [x14, #(0*8+1*32)] // A4[0]
	ins		v31.d[1], x27 // B1[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	ldr		x27, [x11, #(2*8+3*32)] // B1[1]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v10.2d, v26.2d, v30.d[1]

	// load 29 ins 27
	ldr		d29, [x14, #(2*8+1*32)] // A5[0]
	ins		v27.d[1], x23 // A3[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	ldr		x23, [x13, #(3*8+2*32)] // A3[1]
	fmla	v6.2d, v24.2d, v31.d[1]
	fmla	v5.2d, v25.2d, v31.d[0]

	// load 24 ins 28
	ldr		d24, [x9, #(0*8+2*32)] // A0[0]
	ins		v28.d[1], x24 // A4[1]
	fmla	v7.2d, v25.2d, v31.d[1]
	ldr		x24, [x14, #(1*8+2*32)] // A4[1]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v14.2d, v26.2d, v31.d[1]

	// load 25 ins 29
	ldr		d25, [x9, #(2*8+2*32)] // A1[0]
	ins		v29.d[1], x25 // A5[1]
	fmla	v9.2d, v27.2d, v30.d[0]
	ldr		x25, [x14, #(3*8+2*32)] // A5[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v16.2d, v28.2d, v30.d[0]

	// load 26 ins 24
	ldr		d26, [x13, #(0*8+2*32)] // A2[0]
	ins		v24.d[1], x20 // A0[1]
	fmla	v18.2d, v28.2d, v30.d[1]
	ldr		x20, [x9, #(1*8+3*32)] // A0[1]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v19.2d, v29.2d, v30.d[1]

	// load 30 ins 25
	ldr		d30, [x11, #(2*8+0*32)] // B0[0]
	ins		v25.d[1], x21 // A1[1]
	fmla	v13.2d, v27.2d, v31.d[0]
	ldr		x21, [x9, #(3*8+3*32)] // A1[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	fmla	v20.2d, v28.2d, v31.d[0]

	// load 27 ins 30
	ldr		d27, [x13, #(2*8+2*32)] // A3[0]
	ins		v30.d[1], x26 // B0[1]
	fmla	v22.2d, v28.2d, v31.d[1]
	ldr		x26, [x11, #(3*8+1*32)] // B0[1]
	fmla	v21.2d, v29.2d, v31.d[0]
	fmla	v23.2d, v29.2d, v31.d[1]


	// unroll 2

	// load 31 ins 26
	ldr		d31, [x11, #(2*8+2*32)] // B1[0]
	ins		v26.d[1], x22 // A2[1]
	fmla	v0.2d, v24.2d, v30.d[0]
	ldr		x22, [x13, #(1*8+3*32)] // A2[1]
	fmla	v2.2d, v24.2d, v30.d[1]
	fmla	v1.2d, v25.2d, v30.d[0]

	// load 28 ins 31
	ldr		d28, [x14, #(0*8+2*32)] // A4[0]
	ins		v31.d[1], x27 // B1[1]
	fmla	v3.2d, v25.2d, v30.d[1]
	ldr		x27, [x11, #(3*8+3*32)] // B1[1]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v10.2d, v26.2d, v30.d[1]

	// load 29 ins 27
	ldr		d29, [x14, #(2*8+2*32)] // A5[0]
	ins		v27.d[1], x23 // A3[1]
	fmla	v4.2d, v24.2d, v31.d[0]
	ldr		x23, [x13, #(3*8+3*32)] // A3[1]
	fmla	v6.2d, v24.2d, v31.d[1]
	fmla	v5.2d, v25.2d, v31.d[0]

	// load 24 ins 28
	ldr		d24, [x9, #(0*8+3*32)] // A0[0]
	ins		v28.d[1], x24 // A4[1]
	fmla	v7.2d, v25.2d, v31.d[1]
	ldr		x24, [x14, #(1*8+3*32)] // A4[1]
	fmla	v12.2d, v26.2d, v31.d[0]
	fmla	v14.2d, v26.2d, v31.d[1]

	// load 25 ins 29
	ldr		d25, [x9, #(2*8+3*32)] // A1[0]
	ins		v29.d[1], x25 // A5[1]
	fmla	v9.2d, v27.2d, v30.d[0]
	ldr		x25, [x14, #(3*8+3*32)] // A5[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	fmla	v16.2d, v28.2d, v30.d[0]

	// load 26 ins 24
	ldr		d26, [x13, #(0*8+3*32)] // A2[0]
	ins		v24.d[1], x20 // A0[1]
	fmla	v18.2d, v28.2d, v30.d[1]
//	ldr		x20, [x9, #(1*8+4*32)] // A0[1]
	fmla	v17.2d, v29.2d, v30.d[0]
	fmla	v19.2d, v29.2d, v30.d[1]

	// load 30 ins 25
	ldr		d30, [x11, #(3*8+0*32)] // B0[0]
	ins		v25.d[1], x21 // A1[1]
	fmla	v13.2d, v27.2d, v31.d[0]
//	ldr		x21, [x9, #(3*8+4*32)] // A1[1]
	fmla	v15.2d, v27.2d, v31.d[1]
	fmla	v20.2d, v28.2d, v31.d[0]

	// load 27 ins 30
	ldr		d27, [x13, #(2*8+3*32)] // A3[0]
	ins		v30.d[1], x26 // B0[1]
	fmla	v22.2d, v28.2d, v31.d[1]
//	ldr		x26, [x28, #(0*8+1*32)] // B0[1]
	fmla	v21.2d, v29.2d, v31.d[0]
	fmla	v23.2d, v29.2d, v31.d[1]


	// unroll 3

	// load 31 ins 26
	ldr		d31, [x11, #(3*8+2*32)] // B1[0]
	ins		v26.d[1], x22 // A2[1]
	fmla	v0.2d, v24.2d, v30.d[0]
//	ldr		x22, [x13, #(1*8+4*32)] // A2[1]
	fmla	v2.2d, v24.2d, v30.d[1]
	add		x9, x9, #(4*32) // A_0
	fmla	v1.2d, v25.2d, v30.d[0]

	// load 28 ins 31
	ldr		d28, [x14, #(0*8+3*32)] // A4[0]
	ins		v31.d[1], x27 // B1[1]
	fmla	v3.2d, v25.2d, v30.d[1]
//	ldr		x27, [x28, #(0*8+3*32)] // B1[1]
	fmla	v8.2d, v26.2d, v30.d[0]
	fmla	v10.2d, v26.2d, v30.d[1]

	// load 29 ins 27
	ldr		d29, [x14, #(2*8+3*32)] // A5[0]
	ins		v27.d[1], x23 // A3[1]
	fmla	v4.2d, v24.2d, v31.d[0]
//	ldr		x23, [x13, #(3*8+4*32)] // A3[1]
	fmla	v6.2d, v24.2d, v31.d[1]
	sub		w8, w8, #4
	fmla	v5.2d, v25.2d, v31.d[0]

	// load 24 ins 28
//	ldr		d24, [x9, #(0*8+0*32)] // A0[0]
	ins		v28.d[1], x24 // A4[1]
	fmla	v7.2d, v25.2d, v31.d[1]
//	ldr		x24, [x14, #(1*8+4*32)] // A4[1]
	fmla	v12.2d, v26.2d, v31.d[0]
	add		x13, x13, #(4*32) // A_1
	fmla	v14.2d, v26.2d, v31.d[1]

	// load 25 ins 29
//	ldr		d25, [x9, #(2*8+0*32)] // A1[0]
	ins		v29.d[1], x25 // A5[1]
	fmla	v9.2d, v27.2d, v30.d[0]
//	ldr		x25, [x14, #(3*8+4*32)] // A5[1]
	fmla	v11.2d, v27.2d, v30.d[1]
	add		x11, x11, x12 // B
	fmla	v16.2d, v28.2d, v30.d[0]

	// load 26 ins 24
//	ldr		d26, [x13, #(0*8+0*32)] // A2[0]
//	ins		v24.d[1], x20 // A0[1]
	fmla	v18.2d, v28.2d, v30.d[1]
//	ldr		x20, [x9, #(1*8+1*32)] // A0[1]
	fmla	v17.2d, v29.2d, v30.d[0]
	add		x14, x14, #(4*32) // A_2
	fmla	v19.2d, v29.2d, v30.d[1]

	// load 30 ins 25
//	ldr		d30, [x28, #(0*8+0*32)] // B0[0]
//	ins		v25.d[1], x21 // A1[1]
	fmla	v13.2d, v27.2d, v31.d[0]
//	ldr		x21, [x9, #(3*8+1*32)] // A1[1]
	fmla	v15.2d, v27.2d, v31.d[1]
//	cmp		w8, #4
	fmla	v20.2d, v28.2d, v31.d[0]

	// load 27 ins 30
//	ldr		d27, [x13, #(2*8+0*32)] // A3[0]
//	ins		v30.d[1], x26 // B0[1]
	fmla	v22.2d, v28.2d, v31.d[1]
//	ldr		x26, [x28, #(1*8+1*32)] // B0[1]
	fmla	v21.2d, v29.2d, v31.d[0]
	fmla	v23.2d, v29.2d, v31.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x9, x9, #32
//	sub		x11, x11, #32
//	sub		x13, x13, #32

3: // clean1-up loop

	// unroll 0
	ldr		d28, [x11, #(0*8+0*32)]
	ldr		d29, [x11, #(0*8+1*32)]
	ldr		d30, [x11, #(0*8+2*32)]
	ldr		d31, [x11, #(0*8+3*32)]
	ld1		{v24.2d, v25.2d}, [x9], #32
	ld1		{v26.2d, v27.2d}, [x13], #32
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	fmla	v6.2d, v24.2d, v31.d[0]
	fmla	v7.2d, v25.2d, v31.d[0]
	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v9.2d, v27.2d, v28.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	fmla	v11.2d, v27.2d, v29.d[0]
	fmla	v12.2d, v26.2d, v30.d[0]
	fmla	v13.2d, v27.2d, v30.d[0]
	fmla	v14.2d, v26.2d, v31.d[0]
	fmla	v15.2d, v27.2d, v31.d[0]
	ld1		{v24.2d, v25.2d}, [x14], #32
	fmla	v16.2d, v24.2d, v28.d[0]
	fmla	v17.2d, v25.2d, v28.d[0]
	add		x11, x11, #8
	fmla	v18.2d, v24.2d, v29.d[0]
	fmla	v19.2d, v25.2d, v29.d[0]
	sub		w8, w8, #1
	fmla	v20.2d, v24.2d, v30.d[0]
	fmla	v21.2d, v25.2d, v30.d[0]
	cmp		w8, #0
	fmla	v22.2d, v24.2d, v31.d[0]
	fmla	v23.2d, v25.2d, v31.d[0]

	bgt		3b

2: // return


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_12x4_lib4)
#endif
// end




// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- sda
// x11   <- B
// x12   <- 32*sdb
// w13   <- offsetB
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_GEMM_ADD_NN_12X4_LIB4
#else
	.align	4
	FUN_START(inner_edge_gemm_add_nn_12x4_lib4)
#endif

	cmp		w13, #0
	ble		2f // return

	cmp		w8, #0
	ble		2f // return

	mov		w14, #4
	sub		w15, w14, w13 // 4-offsetB
	cmp		w15, w8
	ble		0f
	mov		w15, w8 // kend=min(k,4-offsetB(
0:
//	movgt	w15, w8 // kend=min(k,4-offsetB(
	
	add		x11, x11, x13, LSL #3 // B + offsetB*sizeof(double)

	add		x14, x9, x10
	add		x16, x14, x10

1:
	ldp		q24, q25, [x9, #0]
	ldp		q26, q27, [x14, #0]
	ldr		d28, [x11, #0]
	ldr		d29, [x11, #32]
	ldr		d30, [x11, #64]
	ldr		d31, [x11, #96]

	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	fmla	v6.2d, v24.2d, v31.d[0]
	fmla	v7.2d, v25.2d, v31.d[0]

	fmla	v8.2d, v26.2d, v28.d[0]
	fmla	v9.2d, v27.2d, v28.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	fmla	v11.2d, v27.2d, v29.d[0]
	fmla	v12.2d, v26.2d, v30.d[0]
	fmla	v13.2d, v27.2d, v30.d[0]
	fmla	v14.2d, v26.2d, v31.d[0]
	fmla	v15.2d, v27.2d, v31.d[0]

	ldp		q24, q25, [x16, #0]

	fmla	v16.2d, v24.2d, v28.d[0]
	fmla	v17.2d, v25.2d, v28.d[0]
	fmla	v18.2d, v24.2d, v29.d[0]
	fmla	v19.2d, v25.2d, v29.d[0]
	fmla	v20.2d, v24.2d, v30.d[0]
	fmla	v21.2d, v25.2d, v30.d[0]
	fmla	v22.2d, v24.2d, v31.d[0]
	fmla	v23.2d, v25.2d, v31.d[0]

	add		x9, x9, #32
	add		x14, x14, #32
	add		x16, x16, #32
	add		x11, x11, #8
	sub		w8, w8, #1

	sub		w15, w15, #1

	cmp		w15, #0
	bgt		1b

	cmp		w8, #0
	ble		2f // return

	add		x11, x11, x12
	sub		x11, x11, #32

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_gemm_add_nn_12x4_lib4)
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
// x9   <- sde
//
// output arguments:
// x8   <- E
// x9   <- sde

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRSM_LLN_ONE_12X4_LIB4
#else
	.align 4
	FUN_START(inner_edge_trsm_lln_one_12x4_lib4)
#endif

	add		x10, x8, x9
	add		x11, x10, x9

	ldp		q24, q25, [x8, #0] // E0[0+4*0]
	ldp		q26, q27, [x10, #0] // E1[0+4*0]
	ldp		q28, q29, [x11, #0] // E2[0+4*0]
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

	ldr		q25, [x8, #48] // E[2+4*1]
	ldp		q26, q27, [x10, #32] // E1[0+4*1]
	ldp		q28, q29, [x11, #32] // E2[0+4*1]
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

	ldr		q25, [x8, #80] // E[2+4*2]
	ldp		q26, q27, [x10, #64] // E1[0+4*2]
	ldp		q28, q29, [x11, #64] // E2[0+4*2]
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

	ldp		q26, q27, [x10, #96] // E1[0+4*3]
	ldp		q28, q29, [x11, #96] // E2[0+4*3]
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

	add		x10, x10, #128
	add		x11, x11, #128

	ldp		q24, q25, [x10, #0] // E1[0+4*0]
	ldp		q26, q27, [x11, #0] // E2[0+4*0]
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

	ldr		q25, [x10, #48] // E1[2+4*1]
	ldp		q26, q27, [x11, #32] // E2[0+4*1]
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

	ldr		q25, [x10, #80] // E1[2+4*2]
	ldp		q26, q27, [x11, #64] // E2[0+4*2]
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

	ldp		q26, q27, [x11, #96] // E2[0+4*3]
	fmls	v16.2d, v26.2d, v9.d[1]
	fmls	v17.2d, v27.2d, v9.d[1]
	fmls	v18.2d, v26.2d, v11.d[1]
	fmls	v19.2d, v27.2d, v11.d[1]
	fmls	v20.2d, v26.2d, v13.d[1]
	fmls	v21.2d, v27.2d, v13.d[1]
	fmls	v22.2d, v26.2d, v15.d[1]
	fmls	v23.2d, v27.2d, v15.d[1]

	add		x11, x11, #128

	ldp		q24, q25, [x11, #0] // E2[0+4*4]
	ins		v24.d[0], xzr
	fmls	v16.2d, v24.2d, v16.d[0]
	fmls	v17.2d, v25.2d, v16.d[0]
	fmls	v18.2d, v24.2d, v18.d[0]
	fmls	v19.2d, v25.2d, v18.d[0]
	fmls	v20.2d, v24.2d, v20.d[0]
	fmls	v21.2d, v25.2d, v20.d[0]
	fmls	v22.2d, v24.2d, v22.d[0]
	fmls	v23.2d, v25.2d, v22.d[0]

	ldr		q25, [x11, #48] // E2[2+4*5]
	fmls	v17.2d, v25.2d, v16.d[1]
	fmls	v19.2d, v25.2d, v18.d[1]
	fmls	v21.2d, v25.2d, v20.d[1]
	fmls	v23.2d, v25.2d, v22.d[1]

	ldr		q25, [x11, #80] // E2[2+4*6]
	ins		v25.d[0], xzr
	fmls	v17.2d, v25.2d, v17.d[0]
	fmls	v19.2d, v25.2d, v19.d[0]
	fmls	v21.2d, v25.2d, v21.d[0]
	fmls	v23.2d, v25.2d, v23.d[0]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsm_lln_one_12x4_lib4)
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
	ldr			d24, [x9, #0] // E_inv[0]
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

	ldr			d24, [x9, #8] // E_inv[1]
	fmul		v2.2d, v2.2d, v24.d[0]
	fmul		v3.2d, v3.2d, v24.d[0]
	fmul		v10.2d, v10.2d, v24.d[0]
	fmul		v11.2d, v11.2d, v24.d[0]
	fmul		v18.2d, v18.2d, v24.d[0]
	fmul		v19.2d, v19.2d, v24.d[0]
	ldr			d24, [x8, #48] // E[2+4*1]
	fmls		v4.2d, v2.2d, v24.d[0]
	fmls		v5.2d, v3.2d, v24.d[0]
	fmls		v12.2d, v10.2d, v24.d[0]
	fmls		v13.2d, v11.2d, v24.d[0]
	fmls		v20.2d, v18.2d, v24.d[0]
	fmls		v21.2d, v19.2d, v24.d[0]
	ldr			d24, [x8, #56] // E[3+4*1]
	fmls		v6.2d, v2.2d, v24.d[0]
	fmls		v7.2d, v3.2d, v24.d[0]
	fmls		v14.2d, v10.2d, v24.d[0]
	fmls		v15.2d, v11.2d, v24.d[0]
	fmls		v22.2d, v18.2d, v24.d[0]
	fmls		v23.2d, v19.2d, v24.d[0]

	ldr			d24, [x9, #16] // E_inv[2]
	fmul		v4.2d, v4.2d, v24.d[0]
	fmul		v5.2d, v5.2d, v24.d[0]
	fmul		v12.2d, v12.2d, v24.d[0]
	fmul		v13.2d, v13.2d, v24.d[0]
	fmul		v20.2d, v20.2d, v24.d[0]
	fmul		v21.2d, v21.2d, v24.d[0]
	ldr			d24, [x8, #88] // E[3+4*1]
	fmls		v6.2d, v4.2d, v24.d[0]
	fmls		v7.2d, v5.2d, v24.d[0]
	fmls		v14.2d, v12.2d, v24.d[0]
	fmls		v15.2d, v13.2d, v24.d[0]
	fmls		v22.2d, v20.2d, v24.d[0]
	fmls		v23.2d, v21.2d, v24.d[0]

	ldr			d24, [x9, #24] // E_inv[2]
	fmul		v6.2d, v6.2d, v24.d[0]
	fmul		v7.2d, v7.2d, v24.d[0]
	fmul		v14.2d, v14.2d, v24.d[0]
	fmul		v15.2d, v15.2d, v24.d[0]
	fmul		v22.2d, v22.2d, v24.d[0]
	fmul		v23.2d, v23.2d, v24.d[0]

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
	ldr			d24, [x9, #0] // E_inv[0]
	fmul		v0.2d, v0.2d, v24.d[0]
	fmul		v1.2d, v1.2d, v24.d[0]
	fmul		v8.2d, v8.2d, v24.d[0]
	fmul		v9.2d, v9.2d, v24.d[0]
	fmul		v16.2d, v16.2d, v24.d[0]
	fmul		v17.2d, v17.2d, v24.d[0]
	cmp		w10, #2
	blt		0f // return

	// second column
	ldr			d24, [x8, #8] // E[1+4*0]
	fmls		v2.2d, v0.2d, v24.d[0]
	fmls		v3.2d, v1.2d, v24.d[0]
	fmls		v10.2d, v8.2d, v24.d[0]
	fmls		v11.2d, v9.2d, v24.d[0]
	fmls		v18.2d, v16.2d, v24.d[0]
	fmls		v19.2d, v17.2d, v24.d[0]
	ldr			d24, [x9, #8] // E_inv[1]
	fmul		v2.2d, v2.2d, v24.d[0]
	fmul		v3.2d, v3.2d, v24.d[0]
	fmul		v10.2d, v10.2d, v24.d[0]
	fmul		v11.2d, v11.2d, v24.d[0]
	fmul		v18.2d, v18.2d, v24.d[0]
	fmul		v19.2d, v19.2d, v24.d[0]
	cmp		w10, #3
	blt		0f // return

	// third column
	ldr			d24, [x8, #16] // E[2+4*0]
	fmls		v4.2d, v0.2d, v24.d[0]
	fmls		v5.2d, v1.2d, v24.d[0]
	fmls		v12.2d, v8.2d, v24.d[0]
	fmls		v13.2d, v9.2d, v24.d[0]
	fmls		v20.2d, v16.2d, v24.d[0]
	fmls		v21.2d, v17.2d, v24.d[0]
	ldr			d24, [x8, #48] // E[2+4*1]
	fmls		v4.2d, v2.2d, v24.d[0]
	fmls		v5.2d, v3.2d, v24.d[0]
	fmls		v12.2d, v10.2d, v24.d[0]
	fmls		v13.2d, v11.2d, v24.d[0]
	fmls		v20.2d, v18.2d, v24.d[0]
	fmls		v21.2d, v19.2d, v24.d[0]
	ldr			d24, [x9, #16] // E_inv[2]
	fmul		v4.2d, v4.2d, v24.d[0]
	fmul		v5.2d, v5.2d, v24.d[0]
	fmul		v12.2d, v12.2d, v24.d[0]
	fmul		v13.2d, v13.2d, v24.d[0]
	fmul		v20.2d, v20.2d, v24.d[0]
	fmul		v21.2d, v21.2d, v24.d[0]
	cmp		w10, #4
	blt		0f // return

	// forth column
	ldr			d24, [x8, #24] // E[3+4*0]
	fmls		v6.2d, v0.2d, v24.d[0]
	fmls		v7.2d, v1.2d, v24.d[0]
	fmls		v14.2d, v8.2d, v24.d[0]
	fmls		v15.2d, v9.2d, v24.d[0]
	fmls		v22.2d, v16.2d, v24.d[0]
	fmls		v23.2d, v17.2d, v24.d[0]
	ldr			d24, [x8, #56] // E[3+4*1]
	fmls		v6.2d, v2.2d, v24.d[0]
	fmls		v7.2d, v3.2d, v24.d[0]
	fmls		v14.2d, v10.2d, v24.d[0]
	fmls		v15.2d, v11.2d, v24.d[0]
	fmls		v22.2d, v18.2d, v24.d[0]
	fmls		v23.2d, v19.2d, v24.d[0]
	ldr			d24, [x8, #88] // E[3+4*1]
	fmls		v6.2d, v4.2d, v24.d[0]
	fmls		v7.2d, v5.2d, v24.d[0]
	fmls		v14.2d, v12.2d, v24.d[0]
	fmls		v15.2d, v13.2d, v24.d[0]
	fmls		v22.2d, v18.2d, v24.d[0]
	fmls		v23.2d, v19.2d, v24.d[0]
	ldr			d24, [x9, #24] // E_inv[2]
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

#if defined(OS_LINUX)
	FUN_END(inner_edge_trsm_rlt_inv_12x4_vs_lib4)
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
//
// output arguments:
// x8   <- E

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRSM_RLT_ONE_12X4_LIB4
#else
	.align 4
	FUN_START(inner_edge_trsm_rlt_one_12x4_lib4)
#endif
	
	// first column
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

	ldr			d24, [x8, #48] // E[2+4*1]
	fmls		v4.2d, v2.2d, v24.d[0]
	fmls		v5.2d, v3.2d, v24.d[0]
	fmls		v12.2d, v10.2d, v24.d[0]
	fmls		v13.2d, v11.2d, v24.d[0]
	fmls		v20.2d, v18.2d, v24.d[0]
	fmls		v21.2d, v19.2d, v24.d[0]
	ldr			d24, [x8, #56] // E[3+4*1]
	fmls		v6.2d, v2.2d, v24.d[0]
	fmls		v7.2d, v3.2d, v24.d[0]
	fmls		v14.2d, v10.2d, v24.d[0]
	fmls		v15.2d, v11.2d, v24.d[0]
	fmls		v22.2d, v18.2d, v24.d[0]
	fmls		v23.2d, v19.2d, v24.d[0]

	ldr			d24, [x8, #88] // E[3+4*1]
	fmls		v6.2d, v4.2d, v24.d[0]
	fmls		v7.2d, v5.2d, v24.d[0]
	fmls		v14.2d, v12.2d, v24.d[0]
	fmls		v15.2d, v13.2d, v24.d[0]
	fmls		v22.2d, v20.2d, v24.d[0]
	fmls		v23.2d, v21.2d, v24.d[0]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsm_rlt_one_12x4_lib4)
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
// w9   <- n1
//
// output arguments:
// x8   <- E
// w9   <- n1

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRSM_RLT_ONE_12X4_VS_LIB4
#else
	.align 4
	FUN_START(inner_edge_trsm_rlt_one_12x4_vs_lib4)
#endif
	
	// first column
	cmp			w9, #2
	blt			0f // return

	// second column
	ldr			d24, [x8, #8] // E[1+4*0]
	fmls		v2.2d, v0.2d, v24.d[0]
	fmls		v3.2d, v1.2d, v24.d[0]
	fmls		v10.2d, v8.2d, v24.d[0]
	fmls		v11.2d, v9.2d, v24.d[0]
	fmls		v18.2d, v16.2d, v24.d[0]
	fmls		v19.2d, v17.2d, v24.d[0]
	cmp			w9, #3
	blt			0f // return

	// third column
	ldr			d24, [x8, #16] // E[2+4*0]
	fmls		v4.2d, v0.2d, v24.d[0]
	fmls		v5.2d, v1.2d, v24.d[0]
	fmls		v12.2d, v8.2d, v24.d[0]
	fmls		v13.2d, v9.2d, v24.d[0]
	fmls		v20.2d, v16.2d, v24.d[0]
	fmls		v21.2d, v17.2d, v24.d[0]
	ldr			d24, [x8, #48] // E[2+4*1]
	fmls		v4.2d, v2.2d, v24.d[0]
	fmls		v5.2d, v3.2d, v24.d[0]
	fmls		v12.2d, v10.2d, v24.d[0]
	fmls		v13.2d, v11.2d, v24.d[0]
	fmls		v20.2d, v18.2d, v24.d[0]
	fmls		v21.2d, v19.2d, v24.d[0]
	cmp			w9, #4
	blt			0f // return

	// forth column
	ldr			d24, [x8, #24] // E[3+4*0]
	fmls		v6.2d, v0.2d, v24.d[0]
	fmls		v7.2d, v1.2d, v24.d[0]
	fmls		v14.2d, v8.2d, v24.d[0]
	fmls		v15.2d, v9.2d, v24.d[0]
	fmls		v22.2d, v16.2d, v24.d[0]
	fmls		v23.2d, v17.2d, v24.d[0]
	ldr			d24, [x8, #56] // E[3+4*1]
	fmls		v6.2d, v2.2d, v24.d[0]
	fmls		v7.2d, v3.2d, v24.d[0]
	fmls		v14.2d, v10.2d, v24.d[0]
	fmls		v15.2d, v11.2d, v24.d[0]
	fmls		v22.2d, v18.2d, v24.d[0]
	fmls		v23.2d, v19.2d, v24.d[0]
	ldr			d24, [x8, #88] // E[3+4*1]
	fmls		v6.2d, v4.2d, v24.d[0]
	fmls		v7.2d, v5.2d, v24.d[0]
	fmls		v14.2d, v12.2d, v24.d[0]
	fmls		v15.2d, v13.2d, v24.d[0]
	fmls		v22.2d, v20.2d, v24.d[0]
	fmls		v23.2d, v21.2d, v24.d[0]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsm_rlt_one_12x4_vs_lib4)
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
	
	fmov		d24, 1.0e+0 // 1.0

	// first column
	ins			v25.d[0], v0.d[0]
	fcmpe		d25, #0.0
	ble			1f
	fsqrt		d25, d25
	fdiv		d26, d24, d25
2:
	str			d26, [x8, #0]
	fmul		v0.2d, v0.2d, v26.d[0]
	fmul		v1.2d, v1.2d, v26.d[0]
	fmul		v8.2d, v8.2d, v26.d[0]
	fmul		v9.2d, v9.2d, v26.d[0]
	fmul		v16.2d, v16.2d, v26.d[0]
	fmul		v17.2d, v17.2d, v26.d[0]
	fmls		v2.2d, v0.2d, v0.d[1]
	fmls		v3.2d, v1.2d, v0.d[1]
	fmls		v10.2d, v8.2d, v0.d[1]
	fmls		v11.2d, v9.2d, v0.d[1]
	fmls		v18.2d, v16.2d, v0.d[1]
	fmls		v19.2d, v17.2d, v0.d[1]
	fmls		v5.2d, v1.2d, v1.d[0]
	fmls		v12.2d, v8.2d, v1.d[0]
	fmls		v13.2d, v9.2d, v1.d[0]
	fmls		v20.2d, v16.2d, v1.d[0]
	fmls		v21.2d, v17.2d, v1.d[0]
	fmls		v7.2d, v1.2d, v1.d[1]
	fmls		v14.2d, v8.2d, v1.d[1]
	fmls		v15.2d, v9.2d, v1.d[1]
	fmls		v22.2d, v16.2d, v1.d[1]
	fmls		v23.2d, v17.2d, v1.d[1]

	// second column
	ins			v25.d[0], v2.d[1]
	fcmpe		d25, #0.0
	ble			3f
	fsqrt		d25, d25
	fdiv		d26, d24, d25
4:
	str			d26, [x8, #8]
	fmul		v2.2d, v2.2d, v26.d[0]
	fmul		v3.2d, v3.2d, v26.d[0]
	fmul		v10.2d, v10.2d, v26.d[0]
	fmul		v11.2d, v11.2d, v26.d[0]
	fmul		v18.2d, v18.2d, v26.d[0]
	fmul		v19.2d, v19.2d, v26.d[0]
	fmls		v5.2d, v3.2d, v3.d[0]
	fmls		v12.2d, v10.2d, v3.d[0]
	fmls		v13.2d, v11.2d, v3.d[0]
	fmls		v20.2d, v18.2d, v3.d[0]
	fmls		v21.2d, v19.2d, v3.d[0]
	fmls		v7.2d, v3.2d, v3.d[1]
	fmls		v14.2d, v10.2d, v3.d[1]
	fmls		v15.2d, v11.2d, v3.d[1]
	fmls		v22.2d, v18.2d, v3.d[1]
	fmls		v23.2d, v19.2d, v3.d[1]

	// third column
	ins			v25.d[0], v5.d[0]
	fcmpe		d25, #0.0
	ble			5f
	fsqrt		d25, d25
	fdiv		d26, d24, d25
6:
	str			d26, [x8, #16]
	fmul		v5.2d, v5.2d, v26.d[0]
	fmul		v12.2d, v12.2d, v26.d[0]
	fmul		v13.2d, v13.2d, v26.d[0]
	fmul		v20.2d, v20.2d, v26.d[0]
	fmul		v21.2d, v21.2d, v26.d[0]
	fmls		v7.2d, v5.2d, v5.d[1]
	fmls		v14.2d, v12.2d, v5.d[1]
	fmls		v15.2d, v13.2d, v5.d[1]
	fmls		v22.2d, v20.2d, v5.d[1]
	fmls		v23.2d, v21.2d, v5.d[1]

	// fourth column
	ins			v25.d[0], v7.d[1]
	fcmpe		d25, #0.0
	ble			7f
	fsqrt		d25, d25
	fdiv		d26, d24, d25
8:
	str			d26, [x8, #24]
	fmul		v7.2d, v7.2d, v26.d[0]
	fmul		v14.2d, v14.2d, v26.d[0]
	fmul		v15.2d, v15.2d, v26.d[0]
	fmul		v22.2d, v22.2d, v26.d[0]
	fmul		v23.2d, v23.2d, v26.d[0]

	b			0f

1:
	fmov		d26, xzr
	b			2b

3:
	fmov		d26, xzr
	b			4b

5:
	fmov		d26, xzr
	b			6b

7:
	fmov		d26, xzr

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
	
	fmov		d24, 1.0e+0 // 1.0

	// first column
	ins			v25.d[0], v0.d[0]
	fcmpe		d25, #0.0
	ble			1f
	fsqrt		d25, d25
	fdiv		d26, d24, d25
2:
	str			d26, [x8, #0]
	fmul		v0.2d, v0.2d, v26.d[0]
	fmul		v1.2d, v1.2d, v26.d[0]
	fmul		v8.2d, v8.2d, v26.d[0]
	fmul		v9.2d, v9.2d, v26.d[0]
	fmul		v16.2d, v16.2d, v26.d[0]
	fmul		v17.2d, v17.2d, v26.d[0]
	cmp		w9, #2
	blt		0f // return

	// second column
	fmls		v2.2d, v0.2d, v0.d[1]
	fmls		v3.2d, v1.2d, v0.d[1]
	fmls		v10.2d, v8.2d, v0.d[1]
	fmls		v11.2d, v9.2d, v0.d[1]
	fmls		v18.2d, v16.2d, v0.d[1]
	fmls		v19.2d, v17.2d, v0.d[1]
	ins			v25.d[0], v2.d[1]
	fcmpe		d25, #0.0
	ble			3f
	fsqrt		d25, d25
	fdiv		d26, d24, d25
4:
	str			d26, [x8, #8]
	fmul		v2.2d, v2.2d, v26.d[0]
	fmul		v3.2d, v3.2d, v26.d[0]
	fmul		v10.2d, v10.2d, v26.d[0]
	fmul		v11.2d, v11.2d, v26.d[0]
	fmul		v18.2d, v18.2d, v26.d[0]
	fmul		v19.2d, v19.2d, v26.d[0]
	cmp		w9, #3
	blt		0f // return

	// third column
	fmls		v5.2d, v1.2d, v1.d[0]
	fmls		v12.2d, v8.2d, v1.d[0]
	fmls		v13.2d, v9.2d, v1.d[0]
	fmls		v20.2d, v16.2d, v1.d[0]
	fmls		v21.2d, v17.2d, v1.d[0]
	fmls		v5.2d, v3.2d, v3.d[0]
	fmls		v12.2d, v10.2d, v3.d[0]
	fmls		v13.2d, v11.2d, v3.d[0]
	fmls		v20.2d, v18.2d, v3.d[0]
	fmls		v21.2d, v19.2d, v3.d[0]
	ins			v25.d[0], v5.d[0]
	fcmpe		d25, #0.0
	ble			5f
	fsqrt		d25, d25
	fdiv		d26, d24, d25
6:
	str			d26, [x8, #16]
	fmul		v5.2d, v5.2d, v26.d[0]
	fmul		v12.2d, v12.2d, v26.d[0]
	fmul		v13.2d, v13.2d, v26.d[0]
	fmul		v20.2d, v20.2d, v26.d[0]
	fmul		v21.2d, v21.2d, v26.d[0]
	cmp		w9, #4
	blt		0f // return

	// fourth column
	fmls		v7.2d, v1.2d, v1.d[1]
	fmls		v14.2d, v8.2d, v1.d[1]
	fmls		v15.2d, v9.2d, v1.d[1]
	fmls		v22.2d, v16.2d, v1.d[1]
	fmls		v23.2d, v17.2d, v1.d[1]
	fmls		v7.2d, v3.2d, v3.d[1]
	fmls		v14.2d, v10.2d, v3.d[1]
	fmls		v15.2d, v11.2d, v3.d[1]
	fmls		v22.2d, v18.2d, v3.d[1]
	fmls		v23.2d, v19.2d, v3.d[1]
	fmls		v7.2d, v5.2d, v5.d[1]
	fmls		v14.2d, v12.2d, v5.d[1]
	fmls		v15.2d, v13.2d, v5.d[1]
	fmls		v22.2d, v20.2d, v5.d[1]
	fmls		v23.2d, v21.2d, v5.d[1]
	ins			v25.d[0], v7.d[1]
	fcmpe		d25, #0.0
	ble			7f
	fsqrt		d25, d25
	fdiv		d26, d24, d25
8:
	str			d26, [x8, #24]
	fmul		v7.2d, v7.2d, v26.d[0]
	fmul		v14.2d, v14.2d, v26.d[0]
	fmul		v15.2d, v15.2d, v26.d[0]
	fmul		v22.2d, v22.2d, v26.d[0]
	fmul		v23.2d, v23.2d, v26.d[0]

	b			0f

1:
	fmov		d26, xzr
	b			2b

3:
	fmov		d26, xzr
	b			4b

5:
	fmov		d26, xzr
	b			6b

7:
	fmov		d26, xzr

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
// v00..v23 <- D
// x8   <- alpha
// x9   <- beta
// x10  <- C
// x11  <- sdc
//
// output arguments:
// x12  <- dirty
// x12  <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_12X4_LIB4
#else
	.align	4
	FUN_START(inner_scale_ab_12x4_lib4)
#endif

	// load alpha
	ld1		{v28.2d}, [x8]

	// load beta
	ld1		{v29.2d}, [x9]

	// A = A*alpha
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

	fcmpe	d29, #0.0
	beq		0f

	add		x12, x10, x11 // 2nd panel
	add		x13, x12, x11

	ld1		{v24.2d, v25.2d, v26.2d, v27.2d}, [x10], #64
	fmla	v0.2d, v24.2d, v29.d[0]
	fmla	v1.2d, v25.2d, v29.d[0]
	fmla	v2.2d, v26.2d, v29.d[0]
	fmla	v3.2d, v27.2d, v29.d[0]

	ld1		{v24.2d, v25.2d, v26.2d, v27.2d}, [x10], #64
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]
	fmla	v6.2d, v26.2d, v29.d[0]
	fmla	v7.2d, v27.2d, v29.d[0]

	ld1		{v24.2d, v25.2d, v26.2d, v27.2d}, [x12], #64
	fmla	v8.2d, v24.2d, v29.d[0]
	fmla	v9.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	fmla	v11.2d, v27.2d, v29.d[0]

	ld1		{v24.2d, v25.2d, v26.2d, v27.2d}, [x12], #64
	fmla	v12.2d, v24.2d, v29.d[0]
	fmla	v13.2d, v25.2d, v29.d[0]
	fmla	v14.2d, v26.2d, v29.d[0]
	fmla	v15.2d, v27.2d, v29.d[0]

	ld1		{v24.2d, v25.2d, v26.2d, v27.2d}, [x13], #64
	fmla	v16.2d, v24.2d, v29.d[0]
	fmla	v17.2d, v25.2d, v29.d[0]
	fmla	v18.2d, v26.2d, v29.d[0]
	fmla	v19.2d, v27.2d, v29.d[0]

	ld1		{v24.2d, v25.2d, v26.2d, v27.2d}, [x13], #64
	fmla	v20.2d, v24.2d, v29.d[0]
	fmla	v21.2d, v25.2d, v29.d[0]
	fmla	v22.2d, v26.2d, v29.d[0]
	fmla	v23.2d, v27.2d, v29.d[0]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_12x4_lib4)
#endif
// end




// subroutine
//
// input arguments:
// x8  <- beta
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

	fneg	v16.2d, v16.2d
	fneg	v17.2d, v17.2d
	fneg	v18.2d, v18.2d
	fneg	v19.2d, v19.2d

	fneg	v20.2d, v20.2d
	fneg	v21.2d, v21.2d
	fneg	v22.2d, v22.2d
	fneg	v23.2d, v23.2d

	fcmpe	d29, #0.0
	beq		0f

	add		x11, x9, x10
	add		x12, x11, x10

	ld1		{v24.2d, v25.2d, v26.2d, v27.2d}, [x9], #64
	fmla	v0.2d, v24.2d, v29.d[0]
	fmla	v1.2d, v25.2d, v29.d[0]
	fmla	v2.2d, v26.2d, v29.d[0]
	fmla	v3.2d, v27.2d, v29.d[0]

	ld1		{v24.2d, v25.2d, v26.2d, v27.2d}, [x9], #64
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]
	fmla	v6.2d, v26.2d, v29.d[0]
	fmla	v7.2d, v27.2d, v29.d[0]

	ld1		{v24.2d, v25.2d, v26.2d, v27.2d}, [x11], #64
	fmla	v8.2d, v24.2d, v29.d[0]
	fmla	v9.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	fmla	v11.2d, v27.2d, v29.d[0]

	ld1		{v24.2d, v25.2d, v26.2d, v27.2d}, [x11], #64
	fmla	v12.2d, v24.2d, v29.d[0]
	fmla	v13.2d, v25.2d, v29.d[0]
	fmla	v14.2d, v26.2d, v29.d[0]
	fmla	v15.2d, v27.2d, v29.d[0]

	ld1		{v24.2d, v25.2d, v26.2d, v27.2d}, [x12], #64
	fmla	v16.2d, v24.2d, v29.d[0]
	fmla	v17.2d, v25.2d, v29.d[0]
	fmla	v18.2d, v26.2d, v29.d[0]
	fmla	v19.2d, v27.2d, v29.d[0]

	ld1		{v24.2d, v25.2d, v26.2d, v27.2d}, [x12], #64
	fmla	v20.2d, v24.2d, v29.d[0]
	fmla	v21.2d, v25.2d, v29.d[0]
	fmla	v22.2d, v26.2d, v29.d[0]
	fmla	v23.2d, v27.2d, v29.d[0]

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
// doc:
// Unit value for coefficients alpha and beta

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M11_12X4_LIB4
#else
	.align	4
	FUN_START(inner_scale_m11_12x4_lib4)
#endif

	add		x10, x8, x9
	add		x11, x10, x9

	ld1		{v24.2d, v25.2d, v26.2d, v27.2d}, [x8], #64
	fsub	v0.2d, v24.2d, v0.2d
	fsub	v1.2d, v25.2d, v1.2d
	fsub	v2.2d, v26.2d, v2.2d
	fsub	v3.2d, v27.2d, v3.2d

	ld1		{v24.2d, v25.2d, v26.2d, v27.2d}, [x8], #64
	fsub	v4.2d, v24.2d, v4.2d
	fsub	v5.2d, v25.2d, v5.2d
	fsub	v6.2d, v26.2d, v6.2d
	fsub	v7.2d, v27.2d, v7.2d

	ld1		{v24.2d, v25.2d, v26.2d, v27.2d}, [x10], #64
	fsub	v8.2d, v24.2d, v8.2d
	fsub	v9.2d, v25.2d, v9.2d
	fsub	v10.2d, v26.2d, v10.2d
	fsub	v11.2d, v27.2d, v11.2d

	ld1		{v24.2d, v25.2d, v26.2d, v27.2d}, [x10], #64
	fsub	v12.2d, v24.2d, v12.2d
	fsub	v13.2d, v25.2d, v13.2d
	fsub	v14.2d, v26.2d, v14.2d
	fsub	v15.2d, v27.2d, v15.2d

	ld1		{v24.2d, v25.2d, v26.2d, v27.2d}, [x11], #64
	fsub	v16.2d, v24.2d, v16.2d
	fsub	v17.2d, v25.2d, v17.2d
	fsub	v18.2d, v26.2d, v18.2d
	fsub	v19.2d, v27.2d, v19.2d

	ld1		{v24.2d, v25.2d, v26.2d, v27.2d}, [x11], #64
	fsub	v20.2d, v24.2d, v20.2d
	fsub	v21.2d, v25.2d, v21.2d
	fsub	v22.2d, v26.2d, v22.2d
	fsub	v23.2d, v27.2d, v23.2d

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m11_12x4_lib4)
#endif
// end




// subroutine
//
// input arguments:
// x8   <- alpha
// x9   <- beta
// x10  <- C
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_TRAN_SCALE_AB_4X12_LIB4
#else
	.align	4
	FUN_START(inner_tran_scale_ab_4x12_lib4)
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

	trn1	v24.2d, v16.2d, v18.2d
	trn2	v18.2d, v16.2d, v18.2d
	trn1	v25.2d, v21.2d, v23.2d
	trn2	v23.2d, v21.2d, v23.2d
	trn1	v26.2d, v17.2d, v19.2d
	trn2	v27.2d, v17.2d, v19.2d
	trn1	v17.2d, v20.2d, v22.2d
	trn2	v19.2d, v20.2d, v22.2d
	mov		v16.16b, v24.16b
	mov		v21.16b, v25.16b
	mov		v20.16b, v26.16b
	mov		v22.16b, v27.16b

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
	fmul	v16.2d, v16.2d, v28.d[0]
	fmul	v17.2d, v17.2d, v28.d[0]
	fmul	v18.2d, v18.2d, v28.d[0]
	fmul	v19.2d, v19.2d, v28.d[0]
	fmul	v20.2d, v20.2d, v28.d[0]
	fmul	v21.2d, v21.2d, v28.d[0]
	fmul	v22.2d, v22.2d, v28.d[0]
	fmul	v23.2d, v23.2d, v28.d[0]

	fcmpe	d29, #0.0
	beq		0f

	ld1		{v24.2d, v25.2d, v26.2d, v27.2d}, [x10], #64
	fmla	v0.2d, v24.2d, v29.d[0]
	fmla	v1.2d, v25.2d, v29.d[0]
	fmla	v2.2d, v26.2d, v29.d[0]
	fmla	v3.2d, v27.2d, v29.d[0]

	ld1		{v24.2d, v25.2d, v26.2d, v27.2d}, [x10], #64
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]
	fmla	v6.2d, v26.2d, v29.d[0]
	fmla	v7.2d, v27.2d, v29.d[0]

	ld1		{v24.2d, v25.2d, v26.2d, v27.2d}, [x10], #64
	fmla	v8.2d, v24.2d, v29.d[0]
	fmla	v9.2d, v25.2d, v29.d[0]
	fmla	v10.2d, v26.2d, v29.d[0]
	fmla	v11.2d, v27.2d, v29.d[0]

	ld1		{v24.2d, v25.2d, v26.2d, v27.2d}, [x10], #64
	fmla	v12.2d, v24.2d, v29.d[0]
	fmla	v13.2d, v25.2d, v29.d[0]
	fmla	v14.2d, v26.2d, v29.d[0]
	fmla	v15.2d, v27.2d, v29.d[0]

	ld1		{v24.2d, v25.2d, v26.2d, v27.2d}, [x10], #64
	fmla	v16.2d, v24.2d, v29.d[0]
	fmla	v17.2d, v25.2d, v29.d[0]
	fmla	v18.2d, v26.2d, v29.d[0]
	fmla	v19.2d, v27.2d, v29.d[0]

	ld1		{v24.2d, v25.2d, v26.2d, v27.2d}, [x10], #64
	fmla	v20.2d, v24.2d, v29.d[0]
	fmla	v21.2d, v25.2d, v29.d[0]
	fmla	v22.2d, v26.2d, v29.d[0]
	fmla	v23.2d, v27.2d, v29.d[0]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_tran_scale_ab_4x12_lib4)
#endif





// subroutine
//
// input arguments:
// x8   <- D
// x9   <- sdd
//
// output arguments:
// x10 -> dirty
// x11 -> dirty

#if MACRO_LEVEL>=1
	.macro INNER_STORE_12X4_LIB4
#else
	.align 4
	FUN_START(inner_store_12x4_lib4)
#endif

	add		x10, x8, x9
	add		x11, x10, x9

	stp		q0, q1, [x8, #0]
	stp		q2, q3, [x8, #32]
	stp		q4, q5, [x8, #64]
	stp		q6, q7, [x8, #96]

	stp		q8, q9, [x10, #0]
	stp		q10, q11, [x10, #32]
	stp		q12, q13, [x10, #64]
	stp		q14, q15, [x10, #96]

	stp		q16, q17, [x11, #0]
	stp		q18, q19, [x11, #32]
	stp		q20, q21, [x11, #64]
	stp		q22, q23, [x11, #96]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_12x4_lib4)
#endif
// end




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

	ldp		q24, q25, [x13, #(0*8+0*32)]
	ldp		q26, q27, [x13, #(0*8+1*32)]
	ldp		q28, q29, [x13, #(0*8+2*32)]
	ldp		q30, q31, [x13, #(0*8+3*32)]

	// 4th row
	ins		v17.d[1], v25.d[1]
	ins		v19.d[1], v27.d[1]
	ins		v21.d[1], v29.d[1]
	ins		v23.d[1], v31.d[1]
	cmp		w10, #11
	bge		1f
	// 3th row
	ins		v17.d[0], v25.d[0]
	ins		v19.d[0], v27.d[0]
	ins		v21.d[0], v29.d[0]
	ins		v23.d[0], v31.d[0]
	cmp		w10, #10
	bge		1f
	// 2nd row
	ins		v16.d[1], v24.d[1]
	ins		v18.d[1], v26.d[1]
	ins		v20.d[1], v28.d[1]
	ins		v22.d[1], v30.d[1]
	cmp		w10, #9
	bge		1f
	// 1st row
	ins		v16.d[0], v24.d[0]
	ins		v18.d[0], v26.d[0]
	ins		v20.d[0], v28.d[0]
	ins		v22.d[0], v30.d[0]

1:
	// 1st col
	stp		q0, q1, [x8, #(0*8+0*32)]
	stp		q8, q9, [x12, #(0*8+0*32)]
	stp		q16, q17, [x13, #(0*8+0*32)]
	cmp		w11, #2
	blt		0f
	// 2nd col
	stp		q2, q3, [x8, #(0*8+1*32)]
	stp		q10, q11, [x12, #(0*8+1*32)]
	stp		q18, q19, [x13, #(0*8+1*32)]
	cmp		w11, #3
	blt		0f
	// 3rd col
	stp		q4, q5, [x8, #(0*8+2*32)]
	stp		q12, q13, [x12, #(0*8+2*32)]
	stp		q20, q21, [x13, #(0*8+2*32)]
	beq		0f
	// 4th col
	stp		q6, q7, [x8, #(0*8+3*32)]
	stp		q14, q15, [x12, #(0*8+3*32)]
	stp		q22, q23, [x13, #(0*8+3*32)]

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

	ldr		q24, [x8, #32]
	ldr		q25, [x8, #112]

	ins		v2.d[0], v24.d[0]
	ins		v7.d[0], v25.d[0]

	add		x10, x8, x9
	add		x11, x10, x9

	stp		q0, q1, [x8, #0]
	stp		q2, q3, [x8, #32]
	str		q5, [x8, #80]
	str		q7, [x8, #112]

	stp		q8, q9, [x10, #0]
	stp		q10, q11, [x10, #32]
	stp		q12, q13, [x10, #64]
	stp		q14, q15, [x10, #96]

	stp		q16, q17, [x11, #0]
	stp		q18, q19, [x11, #32]
	stp		q20, q21, [x11, #64]
	stp		q22, q23, [x11, #96]

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

	ldp		q24, q25, [x13, #(0*8+0*32)]
	ldp		q26, q27, [x13, #(0*8+1*32)]
	ldp		q28, q29, [x13, #(0*8+2*32)]
	ldp		q30, q31, [x13, #(0*8+3*32)]

	// 4th row
	ins		v17.d[1], v25.d[1]
	ins		v19.d[1], v27.d[1]
	ins		v21.d[1], v29.d[1]
	ins		v23.d[1], v31.d[1]
	cmp		w10, #11
	bge		1f
	// 3th row
	ins		v17.d[0], v25.d[0]
	ins		v19.d[0], v27.d[0]
	ins		v21.d[0], v29.d[0]
	ins		v23.d[0], v31.d[0]
	cmp		w10, #10
	bge		1f
	// 2nd row
	ins		v16.d[1], v24.d[1]
	ins		v18.d[1], v26.d[1]
	ins		v20.d[1], v28.d[1]
	ins		v22.d[1], v30.d[1]
	cmp		w10, #9
	bge		1f
	// 1st row
	ins		v16.d[0], v24.d[0]
	ins		v18.d[0], v26.d[0]
	ins		v20.d[0], v28.d[0]
	ins		v22.d[0], v30.d[0]

1:
	ldr		q24, [x8, #32]
	ldr		q25, [x8, #112]

	ins		v2.d[0], v24.d[0]
	ins		v7.d[0], v25.d[0]

	// 1st col
	stp		q0, q1, [x8, #(0*8+0*32)]
	stp		q8, q9, [x12, #(0*8+0*32)]
	stp		q16, q17, [x13, #(0*8+0*32)]
	cmp		w11, #2
	blt		0f
	// 2nd col
	stp		q2, q3, [x8, #(0*8+1*32)]
	stp		q10, q11, [x12, #(0*8+1*32)]
	stp		q18, q19, [x13, #(0*8+1*32)]
	cmp		w11, #3
	blt		0f
	// 3rd col
	str		q5, [x8, #(2*8+2*32)]
	stp		q12, q13, [x12, #(0*8+2*32)]
	stp		q20, q21, [x13, #(0*8+2*32)]
	beq		0f
	// 4th col
	str		q7, [x8, #(2*8+3*32)]
	stp		q14, q15, [x12, #(0*8+3*32)]
	stp		q22, q23, [x13, #(0*8+3*32)]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_12x4_vs_lib4)
#endif





// subroutine
//
// input arguments:
// x8   <- D
// x9   <- sdd
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_U_12X4_LIB4
#else
	.align 4
	FUN_START(inner_store_u_12x4_lib4)
#endif

	add		x10, x8, x9
	add		x11, x10, x9

	stp		q0, q1, [x8, #0]
	stp		q2, q3, [x8, #32]
	stp		q4, q5, [x8, #64]
	stp		q6, q7, [x8, #96]

	stp		q8, q9, [x10, #0]
	stp		q10, q11, [x10, #32]
	stp		q12, q13, [x10, #64]
	stp		q14, q15, [x10, #96]

	str		d16, [x11, #0]
	str		q18, [x11, #32]
	str		q20, [x11, #64]
	str		d21, [x11, #80]
	stp		q22, q23, [x11, #96]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_u_12x4_lib4)
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
	.macro INNER_STORE_U_12X4_VS_LIB4
#else
	.align 4
	FUN_START(inner_store_u_12x4_vs_lib4)
#endif

	add		x12, x8, x9
	add		x13, x12, x9

	cmp		w10, #12
	bge		1f

	ldp		q24, q25, [x13, #(0*8+0*32)]
	ldp		q26, q27, [x13, #(0*8+1*32)]
	ldp		q28, q29, [x13, #(0*8+2*32)]
	ldp		q30, q31, [x13, #(0*8+3*32)]

	// 4th row
	ins		v17.d[1], v25.d[1]
	ins		v19.d[1], v27.d[1]
	ins		v21.d[1], v29.d[1]
	ins		v23.d[1], v31.d[1]
	cmp		w10, #11
	bge		1f
	// 3th row
	ins		v17.d[0], v25.d[0]
	ins		v19.d[0], v27.d[0]
	ins		v21.d[0], v29.d[0]
	ins		v23.d[0], v31.d[0]
	cmp		w10, #10
	bge		1f
	// 2nd row
	ins		v16.d[1], v24.d[1]
	ins		v18.d[1], v26.d[1]
	ins		v20.d[1], v28.d[1]
	ins		v22.d[1], v30.d[1]
	cmp		w10, #9
	bge		1f
	// 1st row
	ins		v16.d[0], v24.d[0]
	ins		v18.d[0], v26.d[0]
	ins		v20.d[0], v28.d[0]
	ins		v22.d[0], v30.d[0]

1:

	// 1st col
	stp		q0, q1, [x8, #0]
	stp		q8, q9, [x12, #0]
	str		d16, [x13, #0]
	cmp		w11, #2
	blt		0f
	// 2nd col
	stp		q2, q3, [x8, #32]
	stp		q10, q11, [x12, #32]
	str		q18, [x13, #32]
	cmp		w11, #3
	blt		0f
	// 3rd col
	stp		q4, q5, [x8, #64]
	stp		q12, q13, [x12, #64]
	str		q20, [x13, #64]
	str		d21, [x13, #80]
	beq		0f
	// 4th col
	stp		q6, q7, [x8, #96]
	stp		q14, q15, [x12, #96]
	stp		q22, q23, [x13, #96]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_u_12x4_vs_lib4)
#endif





// subroutine
//
// input arguments:
// x8   <- D
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X12_LIB4
#else
	.align 4
	FUN_START(inner_store_4x12_lib4)
#endif

	stp		q0, q1, [x8, #(0*8+0*32)]
	stp		q2, q3, [x8, #(0*8+1*32)]
	stp		q4, q5, [x8, #(0*8+2*32)]
	stp		q6, q7, [x8, #(0*8+3*32)]
	stp		q8, q9, [x8, #(0*8+4*32)]
	stp		q10, q11, [x8, #(0*8+5*32)]
	stp		q12, q13, [x8, #(0*8+6*32)]
	stp		q14, q15, [x8, #(0*8+7*32)]
	stp		q16, q17, [x8, #(0*8+8*32)]
	stp		q18, q19, [x8, #(0*8+9*32)]
	stp		q20, q21, [x8, #(0*8+10*32)]
	stp		q22, q23, [x8, #(0*8+11*32)]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4x12_lib4)
#endif





// subroutine
//
// input arguments:
// x8   <- D
// x9  <- km
// x10  <- kn
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X12_VS_LIB4
#else
	.align 4
	FUN_START(inner_store_4x12_vs_lib4)
#endif

	cmp		w9, #1
	blt		0f
	beq		1f
	cmp		w9, #2
	beq		2f
	cmp		w9, #3
	beq		3f
	b		4f

1:
	// 1st-8th col
	str		d0, [x8, #(0*8+0*32)]
	str		d2, [x8, #(0*8+1*32)]
	str		d4, [x8, #(0*8+2*32)]
	str		d6, [x8, #(0*8+3*32)]
	str		d8, [x8, #(0*8+4*32)]
	str		d10, [x8, #(0*8+5*32)]
	str		d12, [x8, #(0*8+6*32)]
	str		d14, [x8, #(0*8+7*32)]
	// 9th col
	str		d16, [x8, #(0*8+8*32)]
	cmp		w10, #10
	blt		0f
	// 10th col
	str		d18, [x8, #(0*8+9*32)]
	cmp		w10, #11
	blt		0f
	// 11th col
	str		d20, [x8, #(0*8+10*32)]
	beq		0f
	// 12th col
	str		d22, [x8, #(0*8+11*32)]

	b		0f

2:
	// 1st-8th col
	str		q0, [x8, #(0*8+0*32)]
	str		q2, [x8, #(0*8+1*32)]
	str		q4, [x8, #(0*8+2*32)]
	str		q6, [x8, #(0*8+3*32)]
	str		q8, [x8, #(0*8+4*32)]
	str		q10, [x8, #(0*8+5*32)]
	str		q12, [x8, #(0*8+6*32)]
	str		q14, [x8, #(0*8+7*32)]
	// 9th col
	str		q16, [x8, #(0*8+8*32)]
	cmp		w10, #10
	blt		0f
	// 10th col
	str		q18, [x8, #(0*8+9*32)]
	cmp		w10, #11
	blt		0f
	// 11th col
	str		q20, [x8, #(0*8+10*32)]
	beq		0f
	// 12th col
	str		q22, [x8, #(0*8+11*32)]

	b		0f

3:
	// 1st-8th col
	str		q0, [x8, #(0*8+0*32)]
	str		d1, [x8, #(2*8+0*32)]
	str		q2, [x8, #(0*8+1*32)]
	str		d3, [x8, #(2*8+1*32)]
	str		q4, [x8, #(0*8+2*32)]
	str		d5, [x8, #(2*8+2*32)]
	str		q6, [x8, #(0*8+3*32)]
	str		d7, [x8, #(2*8+3*32)]
	str		q8, [x8, #(0*8+4*32)]
	str		d9, [x8, #(2*8+4*32)]
	str		q10, [x8, #(0*8+5*32)]
	str		d11, [x8, #(2*8+5*32)]
	str		q12, [x8, #(0*8+6*32)]
	str		d13, [x8, #(2*8+6*32)]
	str		q14, [x8, #(0*8+7*32)]
	str		d15, [x8, #(2*8+7*32)]
	// 9th col
	str		q16, [x8, #(0*8+8*32)]
	str		d17, [x8, #(2*8+8*32)]
	cmp		w10, #10
	blt		0f
	// 10th col
	str		q18, [x8, #(0*8+9*32)]
	str		d19, [x8, #(2*8+9*32)]
	cmp		w10, #11
	blt		0f
	// 11th col
	str		q20, [x8, #(0*8+10*32)]
	str		d21, [x8, #(2*8+10*32)]
	beq		0f
	// 12th col
	str		q22, [x8, #(0*8+11*32)]
	str		d23, [x8, #(2*8+11*32)]

	b		0f

4:
	// 1st-8th col
	stp		q0, q1, [x8, #(0*8+0*32)]
	stp		q2, q3, [x8, #(0*8+1*32)]
	stp		q4, q5, [x8, #(0*8+2*32)]
	stp		q6, q7, [x8, #(0*8+3*32)]
	stp		q8, q9, [x8, #(0*8+4*32)]
	stp		q10, q11, [x8, #(0*8+5*32)]
	stp		q12, q13, [x8, #(0*8+6*32)]
	stp		q14, q15, [x8, #(0*8+7*32)]
	// 9th col
	stp		q16, q17, [x8, #(0*8+8*32)]
	cmp		w10, #10
	blt		0f
	// 10th col
	stp		q18, q19, [x8, #(0*8+9*32)]
	cmp		w10, #11
	blt		0f
	// 11th col
	stp		q20, q21, [x8, #(0*8+10*32)]
	beq		0f
	// 12th col
	stp		q22, q23, [x8, #(0*8+11*32)]

//	b		0f

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4x12_vs_lib4)
#endif





//                                w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8
// void kernel_dgemm_nt_12x4_lib4(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd)

	.align	4
	GLOB(kernel_dgemm_nt_12x4_lib4)
	FUN_START(kernel_dgemm_nt_12x4_lib4)



	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0       // kmax
	mov		x9, x2       // A
	mov		w10, w3      // sda
	lsl		w10, w10, #5 // sda_bit = 32*sda
	mov		x11, x4      // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_12x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // sdc
	lsl		w11, w11, #5 // sdc_bit = 32*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // sdd
	lsl		w9, w9, #5 // 32*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nt_12x4_lib4)
// end





//                                   w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8     sp+16   sp+24
// void kernel_dgemm_nt_12x4_vs_lib4(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, int m1, int n1)

	.align	4
	GLOB(kernel_dgemm_nt_12x4_vs_lib4)
	FUN_START(kernel_dgemm_nt_12x4_vs_lib4)



	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0       // kmax
	mov		x9, x2       // A
	mov		w10, w3      // sda
	lsl		w10, w10, #5 // sda_bit = 32*sda
	mov		x11, x4      // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_12x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // sdc
	lsl		w11, w11, #5 // sdc_bit = 32*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // sdd
	lsl		w9, w9, #5 // 32*sdd
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

	FUN_END(kernel_dgemm_nt_12x4_vs_lib4)
// end





//                                w0        x1             x2         w3       w4           x5         w6       x7            sp+0       sp+8     sp+16      sp+24
// void kernel_dgemm_nn_12x4_lib4(int kmax, double *alpha, double *A, int sda, int offsetB, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd)

	.align	4
	GLOB(kernel_dgemm_nn_12x4_lib4)
	FUN_START(kernel_dgemm_nn_12x4_lib4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x5 // B
	mov		w12, w6 // sdb
	lsl		w12, w12, #5 // 32*sdb
	mov		w13, w4 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMM_ADD_NN_12X4_LIB4
#else
	CALL(inner_edge_gemm_add_nn_12x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_12X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nn_12x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x7 // beta
	ldr		x10, [sp, #(STACKSIZE + 0)] // C
	ldr		w11, [sp, #(STACKSIZE + 8)] // sdc
	lsl		w11, w11, #5 // 32*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 16)] // D
	ldr		w9, [sp, #(STACKSIZE + 24)] // sdd
	lsl		w9, w9, #5 // 32*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nn_12x4_lib4)





//                                   w0        x1             x2         w3       w4           x5         w6       x7            sp+0       sp+8     sp+16      sp+24    sp+32   sp+40
// void kernel_dgemm_nn_12x4_vs_lib4(int kmax, double *alpha, double *A, int sda, int offsetB, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd, int m1, int n1)

	.align	4
	GLOB(kernel_dgemm_nn_12x4_vs_lib4)
	FUN_START(kernel_dgemm_nn_12x4_vs_lib4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x5 // B
	mov		w12, w6 // sdb
	lsl		w12, w12, #5 // 32*sdb
	mov		w13, w4 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMM_ADD_NN_12X4_LIB4
#else
	CALL(inner_edge_gemm_add_nn_12x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_12X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nn_12x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x7 // beta
	ldr		x10, [sp, #(STACKSIZE + 0)] // C
	ldr		w11, [sp, #(STACKSIZE + 8)] // sdc
	lsl		w11, w11, #5 // 32*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 16)] // D
	ldr		w9, [sp, #(STACKSIZE + 24)] // sdd
	lsl		w9, w9, #5 // 32*sdd
	ldr		w10, [sp, #(STACKSIZE + 32)] // m1
	ldr		w11, [sp, #(STACKSIZE + 40)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nn_12x4_vs_lib4)





//                                  w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8
// void kernel_dsyrk_nt_l_12x4_lib4(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd)

	.align	4
	GLOB(kernel_dsyrk_nt_l_12x4_lib4)
	FUN_START(kernel_dsyrk_nt_l_12x4_lib4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B

#if MACRO_LEVEL>=2
//	INNER_KERNEL_SYRK_L_ADD_NT_12X4_LIB4
	INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
//	CALL(inner_kernel_syrk_l_add_nt_12x4_lib4)
	CALL(inner_kernel_gemm_add_nt_12x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // C
	lsl		w11, w11, #5 // 32*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // sdd
	lsl		w9, w9, #5 // 32*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_L_12X4_LIB4
#else
	CALL(inner_store_l_12x4_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dsyrk_nt_l_12x4_lib4)





//                                     w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8     sp+16   sp+24
// void kernel_dsyrk_nt_l_12x4_vs_lib4(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, int m1, int n1)

	.align	4
	GLOB(kernel_dsyrk_nt_l_12x4_vs_lib4)
	FUN_START(kernel_dsyrk_nt_l_12x4_vs_lib4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x4 // B

#if MACRO_LEVEL>=2
//	INNER_KERNEL_SYRK_L_ADD_NT_12X4_LIB4
	INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
//	CALL(inner_kernel_syrk_l_add_nt_12x4_lib4)
	CALL(inner_kernel_gemm_add_nt_12x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // C
	lsl		w11, w11, #5 // 32*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // sdd
	lsl		w9, w9, #5 // 32*sdd
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

	FUN_END(kernel_dsyrk_nt_l_12x4_vs_lib4)





//                                  w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8
// void kernel_dsyrk_nt_u_12x4_lib4(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd)

	.align	4
	GLOB(kernel_dsyrk_nt_u_12x4_lib4)
	FUN_START(kernel_dsyrk_nt_u_12x4_lib4)
	


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
	mov		w11, w7 // C
	lsl		w11, w11, #5 // 32*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // sdd
	lsl		w9, w9, #5 // 32*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_U_12X4_LIB4
#else
	CALL(inner_store_u_12x4_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dsyrk_nt_u_12x4_lib4)





//                                   w0        x1             x2         w3       x4         x5            x6         w7       sp+0       sp+8     sp+16   sp+24
// void kernel_dsyrk_nt_u_12x4_vs_lib4(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, int m1, int n1)

	.align	4
	GLOB(kernel_dsyrk_nt_u_12x4_vs_lib4)
	FUN_START(kernel_dsyrk_nt_u_12x4_vs_lib4)
	


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
	mov		w11, w7 // C
	lsl		w11, w11, #5 // 32*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // sdd
	lsl		w9, w9, #5 // 32*sdd
	ldr		w10, [sp, #(STACKSIZE + 16)] // m1
	ldr		w11, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_U_12X4_VS_LIB4
#else
	CALL(inner_store_u_12x4_vs_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dsyrk_nt_u_12x4_vs_lib4)





//                                  w0        x1             x2         w3       w4           x5         w6       x7            sp+0       sp+8     sp+16      sp+24
// void kernel_dsyrk_nn_u_12x4_lib4(int kmax, double *alpha, double *A, int sda, int offsetB, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd)

	.align	4
	GLOB(kernel_dsyrk_nn_u_12x4_lib4)
	FUN_START(kernel_dsyrk_nn_u_12x4_lib4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x5 // B
	mov		w12, w6 // sdb
	lsl		w12, w12, #5 // 32*sdb
	mov		w13, w4 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMM_ADD_NN_12X4_LIB4
#else
	CALL(inner_edge_gemm_add_nn_12x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_12X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nn_12x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x7 // beta
	ldr		x10, [sp, #(STACKSIZE + 0)] // C
	ldr		w11, [sp, #(STACKSIZE + 8)] // sdc
	lsl		w11, w11, #5 // 32*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 16)] // D
	ldr		w9, [sp, #(STACKSIZE + 24)] // sdd
	lsl		w9, w9, #5 // 32*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_U_12X4_LIB4
#else
	CALL(inner_store_u_12x4_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dsyrk_nn_u_12x4_lib4)





//                                     w0        x1             x2         w3       w4           x5         w6       x7            sp+0       sp+8     sp+16      sp+24    sp+32   sp+40
// void kernel_dsyrk_nn_u_12x4_vs_lib4(int kmax, double *alpha, double *A, int sda, int offsetB, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd, int m1, int n1)

	.align	4
	GLOB(kernel_dsyrk_nn_u_12x4_vs_lib4)
	FUN_START(kernel_dsyrk_nn_u_12x4_vs_lib4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x5 // B
	mov		w12, w6 // sdb
	lsl		w12, w12, #5 // 32*sdb
	mov		w13, w4 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMM_ADD_NN_12X4_LIB4
#else
	CALL(inner_edge_gemm_add_nn_12x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_12X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nn_12x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x7 // beta
	ldr		x10, [sp, #(STACKSIZE + 0)] // C
	ldr		w11, [sp, #(STACKSIZE + 8)] // sdc
	lsl		w11, w11, #5 // 32*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 16)] // D
	ldr		w9, [sp, #(STACKSIZE + 24)] // sdd
	lsl		w9, w9, #5 // 32*sdd
	ldr		w10, [sp, #(STACKSIZE + 32)] // m1
	ldr		w11, [sp, #(STACKSIZE + 40)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_U_12X4_VS_LIB4
#else
	CALL(inner_store_u_12x4_vs_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dsyrk_nn_u_12x4_vs_lib4)





//                                       w0        x1         w2        x3        x4            x5         w6       x7         sp+0     sp+8       sp+16
// void kernel_dtrsm_nt_rl_inv_12x4_lib4(int kmax, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, double *E, double *inv_diag_E);

	.align	4
	GLOB(kernel_dtrsm_nt_rl_inv_12x4_lib4)
	FUN_START(kernel_dtrsm_nt_rl_inv_12x4_lib4)



	PROLOGUE



	ZERO_ACC



	// call inner kernel dgemm nt
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
	mov		w10, w6 // sdc
	lsl		w10, w10, #5 // 32*sdc

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
	lsl		w9, w9, #5 // 32*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_12x4_lib4)





//                                          w0        x1         w2        x3        x4            x5         w6       x7         sp+0     sp+8       sp+16               sp+24   sp+32
// void kernel_dtrsm_nt_rl_inv_12x4_vs_lib4(int kmax, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, double *E, double *inv_diag_E, int m1, int n1);

	.align	4
	GLOB(kernel_dtrsm_nt_rl_inv_12x4_vs_lib4)
	FUN_START(kernel_dtrsm_nt_rl_inv_12x4_vs_lib4)



	PROLOGUE



	ZERO_ACC



	// call inner kernel dgemm nt
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
	mov		w10, w6 // sdc
	lsl		w10, w10, #5 // 32*sdc

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
	lsl		w9, w9, #5 // 32*sdd
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

	FUN_END(kernel_dtrsm_nt_rl_inv_12x4_vs_lib4)





//                                       w0        x1         w2        x3        x4            x5         w6       x7         sp+0     sp+8
// void kernel_dtrsm_nt_rl_one_12x4_lib4(int kmax, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, double *E);

	.align	4
	GLOB(kernel_dtrsm_nt_rl_one_12x4_lib4)
	FUN_START(kernel_dtrsm_nt_rl_one_12x4_lib4)



	PROLOGUE



	ZERO_ACC



	// call inner kernel dgemm nt
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
	mov		w10, w6 // sdc
	lsl		w10, w10, #5 // 32*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 8)] // E

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_ONE_12X4_LIB4
#else
	CALL(inner_edge_trsm_rlt_one_12x4_lib4)
#endif



	// store
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // sdc
	lsl		w9, w9, #5 // 32*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nt_rl_one_12x4_lib4)





//                                          w0        x1         w2        x3        x4            x5         w6       x7         sp+0     sp+8       sp+16   sp+24
// void kernel_dtrsm_nt_rl_one_12x4_vs_lib4(int kmax, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, double *E, int m1, int n1);

	.align	4
	GLOB(kernel_dtrsm_nt_rl_one_12x4_vs_lib4)
	FUN_START(kernel_dtrsm_nt_rl_one_12x4_vs_lib4)



	PROLOGUE



	ZERO_ACC



	// call inner kernel dgemm nt
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
	mov		w10, w6 // sdc
	lsl		w10, w10, #5 // 32*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 8)] // E
	ldr		w9, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_ONE_12X4_VS_LIB4
#else
	CALL(inner_edge_trsm_rlt_one_12x4_vs_lib4)
#endif



	// store
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // sdd
	lsl		w9, w9, #5 // 32*sdd
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

	FUN_END(kernel_dtrsm_nt_rl_one_12x4_vs_lib4)





//                                       w0        x1         w2        x3        w4       x5            x6         w7       sp+0       sp+8     sp+16      sp+24
// void kernel_dtrsm_nn_ll_one_12x4_lib4(int kmax, double *A, int sda, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd, double *E, int sde);

	.align	4
	GLOB(kernel_dtrsm_nn_ll_one_12x4_lib4)
	FUN_START(kernel_dtrsm_nn_ll_one_12x4_lib4)



	PROLOGUE



	ZERO_ACC



	// call inner kernel dgemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x3 // B
	mov		w12, w4 // sdb
	lsl		w12, w12, #5 // 32*sdb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_12X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nn_12x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x5 // beta
	mov		x9, x6 // C
	mov		w10, w7 // sdc
	lsl		w10, w10, #5 // 32*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 16)] // E
	ldr		w9, [sp, #(STACKSIZE + 24)] // sdr
	lsl		w9, w9, #5 // 32*sdc

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_LLN_ONE_12X4_LIB4
#else
	CALL(inner_edge_trsm_lln_one_12x4_lib4)
#endif



	// store
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // sdd
	lsl		w9, w9, #5 // 32*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nn_ll_one_12x4_lib4)





//                                          w0        x1         w2        x3        w4       x5            x6         w7       sp+0       sp+8     sp+16      sp+24    sp+32   sp+40
// void kernel_dtrsm_nn_ll_one_12x4_vs_lib4(int kmax, double *A, int sda, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd, double *E, int sde, int m1, int n1);

	.align	4
	GLOB(kernel_dtrsm_nn_ll_one_12x4_vs_lib4)
	FUN_START(kernel_dtrsm_nn_ll_one_12x4_vs_lib4)



	PROLOGUE



	ZERO_ACC



	// call inner kernel dgemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x3 // B
	mov		w12, w4 // sdb
	lsl		w12, w12, #5 // 32*sdb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_12X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nn_12x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x5 // beta
	mov		x9, x6 // C
	mov		w10, w7 // sdc
	lsl		w10, w10, #5 // 32*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 16)] // E
	ldr		w9, [sp, #(STACKSIZE + 24)] // sdr
	lsl		w9, w9, #5 // 32*sdc

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_LLN_ONE_12X4_LIB4
#else
	CALL(inner_edge_trsm_lln_one_12x4_lib4)
#endif



	// store
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // sdd
	lsl		w9, w9, #5 // 32*sdd
	ldr		w10, [sp, #(STACKSIZE + 32)] // m1
	ldr		w11, [sp, #(STACKSIZE + 40)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nn_ll_one_12x4_vs_lib4)





//                                   w0        x1         w2        x3        x4         w5       x6         w7       sp+0
// void kernel_dpotrf_nt_l_12x4_lib4(int kmax, double *A, int sda, double *B, double *C, int sdc, double *D, int sdd, double *inv_diag_D);

	.align	4
	GLOB(kernel_dpotrf_nt_l_12x4_lib4)
	FUN_START(kernel_dpotrf_nt_l_12x4_lib4)



	PROLOGUE



	ZERO_ACC



	// call inner kernel dsyrk l nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x3 // B

#if MACRO_LEVEL>=2
//	INNER_KERNEL_SYRK_L_ADD_NT_12X4_LIB4
	INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
//	CALL(inner_kernel_syrk_l_add_nt_12x4_lib4)
	CALL(inner_kernel_gemm_add_nt_12x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x4 // C
	mov		w9, w5 // sdc
	lsl		w9, w9, #5 // 32*sdc

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
	lsl		w9, w9, #5 // 32*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_L_12X4_LIB4
#else
	CALL(inner_store_l_12x4_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dpotrf_nt_l_12x4_lib4)




//                                      w0        x1         w2        x3        x4         w5       x6         w7       sp+0                sp+8    sp+16
// void kernel_dpotrf_nt_l_12x4_vs_lib4(int kmax, double *A, int sda, double *B, double *C, int sdc, double *D, int sdd, double *inv_diag_D, int m1, int n1);

	.align	4
	GLOB(kernel_dpotrf_nt_l_12x4_vs_lib4)
	FUN_START(kernel_dpotrf_nt_l_12x4_vs_lib4)



	PROLOGUE



	ZERO_ACC



	// call inner kernel dsyrk l nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x3 // B

#if MACRO_LEVEL>=2
//	INNER_KERNEL_SYRK_L_ADD_NT_12X4_LIB4
	INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
//	CALL(inner_kernel_syrk_l_add_nt_12x4_lib4)
	CALL(inner_kernel_gemm_add_nt_12x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x4 // C
	mov		w9, w5 // sdc
	lsl		w9, w9, #5 // 32*sdc

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
	lsl		w9, w9, #5 // 32*sdd
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

	FUN_END(kernel_dpotrf_nt_l_12x4_vs_lib4)





//                                w0        x1             x2         x3         w4       x5            x6         x7
// void kernel_dgemm_nt_4x12_lib4(int kmax, double *alpha, double *A, double *B, int sda, double *beta, double *C, double *D)

	.align	4
	GLOB(kernel_dgemm_nt_4x12_lib4)
	FUN_START(kernel_dgemm_nt_4x12_lib4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x3 // B
	mov		w10, w4 // sdb
	lsl		w10, w10, #5 // 32*sdb
	mov		x11, x2 // A

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_12x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C

#if MACRO_LEVEL>=1
	INNER_TRAN_SCALE_AB_4X12_LIB4
#else
	CALL(inner_tran_scale_ab_4x12_lib4)
#endif



	// store n
	mov		x8, x7 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_LIB4
#else
	CALL(inner_store_4x12_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nt_4x12_lib4)





//                                   w0        x1             x2         x3         w4       x5            x6         x7         sp+0    sp+8
// void kernel_dgemm_nt_4x12_vs_lib4(int kmax, double *alpha, double *A, double *B, int sda, double *beta, double *C, double *D, int m1, int n1)

	.align	4
	GLOB(kernel_dgemm_nt_4x12_vs_lib4)
	FUN_START(kernel_dgemm_nt_4x12_vs_lib4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x3 // B
	mov		w10, w4 // sdb
	lsl		w10, w10, #5 // 32*sdb
	mov		x11, x2 // A

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_12x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C

#if MACRO_LEVEL>=1
	INNER_TRAN_SCALE_AB_4X12_LIB4
#else
	CALL(inner_tran_scale_ab_4x12_lib4)
#endif



	// store n
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // m1
	ldr		w10, [sp, #(STACKSIZE + 8)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_VS_LIB4
#else
	CALL(inner_store_4x12_vs_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nt_4x12_vs_lib4)





//#if defined(BLAS_API)
#if ( defined(BLAS_API) | ( defined(LA_HIGH_PERFORMANCE) & defined(MF_COLMAJ) ) )

#include "kernel_dgemm_12x4_lib.S"

#endif

