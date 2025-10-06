/**************************************************************************************************
*                                                                                                 *
* This file is part of BLASFEO.                                                                   *
*                                                                                                 *
* BLASFEO -- BLAS For Embedded Optimization.                                                      *
* Copyright (C) 2020 by Gianluca Frison.                                                          *
* All rights reserved.                                                                            *
*                                                                                                 *
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



/* often not needed to save all in such kernels !!!!!!!!! */

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
	fmov    d7, d0

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
.endm

#endif





	.text





//                         w0     x1
// void kernel_dvecld_inc1(int k, double *x)

	.align	4
	GLOB(kernel_dvecld_inc1)
	FUN_START(kernel_dvecld_inc1)
	


	// not needed
//	PROLOGUE



	// call inner kernel
	mov		w8, w0 // k
	mov		x9, x1 // x


	// early return
	cmp		w8, #0
	ble		2f // return

	// consider clean up loop
	cmp		w8, #15
	ble		0f


	// main loop
1:

	ldp		q0, q1, [x9, #0]
	ldp		q2, q3, [x9, #32]
	ldp		q4, q5, [x9, #64]
	ldp		q6, q7, [x9, #96]

	sub		w8, w8, #16
	add		x9, x9, #128

	cmp		w8, #15
	bgt		1b

0:

	// consider clean up loop
	cmp		w8, #3
	ble		0f


	// main loop
1:

	ldp		q0, q1, [x9, #0]

	sub		w8, w8, #4
	add		x9, x9, #32

	cmp		w8, #3
	bgt		1b

0:

	cmp		w8, #0
	ble		2f // return


	// clean up loop
1:

	ldr		d0, [x9, #0]

	sub		w8, w8, #1
	add		x9, x9, #8

	cmp		w8, #0
	bgt		1b



2:

	// not needed
//	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dvecld_inc1)





//                         w0     x1         x2
// void kernel_dveccp_inc1(int k, double *x, double *y)

	.align	4
	GLOB(kernel_dveccp_inc1)
	FUN_START(kernel_dveccp_inc1)
	


	// not needed
//	PROLOGUE



	// call inner kernel
	mov		w8, w0 // k
	mov		x9, x1 // x
	mov		x10, x2 // y


	// early return
	cmp		w8, #0
	ble		2f // return

	// consider clean up loop
	cmp		w8, #15
	ble		0f


	// main loop
1:

	ldp		q0, q1, [x9, #0]
	stp		q0, q1, [x10, #0]
	ldp		q0, q1, [x9, #32]
	stp		q0, q1, [x10, #32]
	ldp		q0, q1, [x9, #64]
	stp		q0, q1, [x10, #64]
	ldp		q0, q1, [x9, #96]
	stp		q0, q1, [x10, #96]

	sub		w8, w8, #16
	add		x9, x9, #128
	add		x10, x10, #128

	cmp		w8, #15
	bgt		1b

0:

	// consider clean up loop
	cmp		w8, #3
	ble		0f


	// main loop
1:

	ldp		q0, q1, [x9, #0]
	stp		q0, q1, [x10, #0]

	sub		w8, w8, #4
	add		x9, x9, #32
	add		x10, x10, #32

	cmp		w8, #3
	bgt		1b

0:

	cmp		w8, #0
	ble		2f // return


	// clean up loop
1:

	ldr		d0, [x9, #0]
	str		d0, [x10, #0]

	sub		w8, w8, #1
	add		x9, x9, #8
	add		x10, x10, #8

	cmp		w8, #0
	bgt		1b



2:

	// not needed
//	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dveccp_inc1)





