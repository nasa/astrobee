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

#endif






	.text





//                              w0        x1         w2       x3         w4
// void kernel_dpack_nn_12_lib4(int kmax, double *A, int lda, double *C, int sdc)

	.align	4
	GLOB(kernel_dpack_nn_12_lib4)
	FUN_START(kernel_dpack_nn_12_lib4)
	


	PROLOGUE



	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // lda
	lsl		w10, w10, #3 // 8*lda
	mov		x11, x3 // A
	mov		w12, w4 // sdc
	lsl		w12, w12, #5 // 32*sdc


	cmp		w8, #0
	ble		0f

	add		x13, x11, x12
	add		x14, x13, x12

	lsl		w15, w10, #2 // 4*(lda*sizeof(double))
	add		x16, x15, #32
	add		x17, x15, #64

	cmp		w8, #3
	ble		2f


1: // main loop
	ldp		q0, q1, [x9, #0]
	prfm	PLDL1KEEP, [x9, x15]
	stp		q0, q1, [x11, #0]
	ldp		q0, q1, [x9, #32]
	prfm	PLDL1KEEP, [x9, x16]
	stp		q0, q1, [x13, #0]
	ldp		q0, q1, [x9, #64]
	prfm	PLDL1KEEP, [x9, x17]
	add		x9, x9, x10
	stp		q0, q1, [x14, #0]

	ldp		q0, q1, [x9, #0]
	prfm	PLDL1KEEP, [x9, x15]
	stp		q0, q1, [x11, #32]
	ldp		q0, q1, [x9, #32]
	prfm	PLDL1KEEP, [x9, x16]
	stp		q0, q1, [x13, #32]
	ldp		q0, q1, [x9, #64]
	prfm	PLDL1KEEP, [x9, x17]
	add		x9, x9, x10
	stp		q0, q1, [x14, #32]

	ldp		q0, q1, [x9, #0]
	prfm	PLDL1KEEP, [x9, x15]
	stp		q0, q1, [x11, #64]
	ldp		q0, q1, [x9, #32]
	prfm	PLDL1KEEP, [x9, x16]
	stp		q0, q1, [x13, #64]
	ldp		q0, q1, [x9, #64]
	prfm	PLDL1KEEP, [x9, x17]
	add		x9, x9, x10
	stp		q0, q1, [x14, #64]

	ldp		q0, q1, [x9, #0]
	prfm	PLDL1KEEP, [x9, x15]
	stp		q0, q1, [x11, #96]
	ldp		q0, q1, [x9, #32]
	prfm	PLDL1KEEP, [x9, x16]
	stp		q0, q1, [x13, #96]
	ldp		q0, q1, [x9, #64]
	prfm	PLDL1KEEP, [x9, x17]
	add		x9, x9, x10
	stp		q0, q1, [x14, #96]

	sub		w8, w8, #4
	add		x11, x11, #128
	add		x13, x13, #128
	add		x14, x14, #128

	cmp		w8, #3
	bgt		1b


	cmp		w8, #0
	ble		0f


2: // clean-up loop
	ldp		q0, q1, [x9, #0]
	stp		q0, q1, [x11, #0]
	ldp		q0, q1, [x9, #32]
	stp		q0, q1, [x13, #0]
	ldp		q0, q1, [x9, #64]
	add		x9, x9, x10
	stp		q0, q1, [x14, #0]

	sub		w8, w8, #1
	add		x11, x11, #32
	add		x13, x13, #32
	add		x14, x14, #32

	cmp		w8, #0
	bgt		2b


0: // return

	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dpack_nn_12_lib4)





//                             w0        x1         w2       x3         w4
// void kernel_dpack_nn_8_lib4(int kmax, double *A, int lda, double *C, int sdc)

	.align	4
	GLOB(kernel_dpack_nn_8_lib4)
	FUN_START(kernel_dpack_nn_8_lib4)
	


	PROLOGUE



	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // lda
	lsl		w10, w10, #3 // 8*lda
	mov		x11, x3 // A
	mov		w12, w4 // sdc
	lsl		w12, w12, #5 // 32*sdc


	cmp		w8, #0
	ble		0f

	add		x13, x11, x12

	lsl		w14, w10, #2 // 4*(lda*sizeof(double))
	add		x15, x14, #32

	cmp		w8, #3
	ble		2f


1: // main loop
	ldp		q0, q1, [x9, #0]
	prfm	PLDL1KEEP, [x9, x14]
	stp		q0, q1, [x11, #0]
//	prfm	PLDL1KEEP, [x11, #128]
	ldp		q0, q1, [x9, #32]
	prfm	PLDL1KEEP, [x9, x15]
	add		x9, x9, x10
	stp		q0, q1, [x13, #0]
//	prfm	PLDL1KEEP, [x13, #128]

	ldp		q0, q1, [x9, #0]
	prfm	PLDL1KEEP, [x9, x14]
	stp		q0, q1, [x11, #32]
	ldp		q0, q1, [x9, #32]
	prfm	PLDL1KEEP, [x9, x15]
	add		x9, x9, x10
	stp		q0, q1, [x13, #32]

	ldp		q0, q1, [x9, #0]
	prfm	PLDL1KEEP, [x9, x14]
	stp		q0, q1, [x11, #64]
//	prfm	PLDL1KEEP, [x11, #192]
	ldp		q0, q1, [x9, #32]
	prfm	PLDL1KEEP, [x9, x15]
	add		x9, x9, x10
	stp		q0, q1, [x13, #64]
//	prfm	PLDL1KEEP, [x13, #192]

	ldp		q0, q1, [x9, #0]
	prfm	PLDL1KEEP, [x9, x14]
	stp		q0, q1, [x11, #96]
	ldp		q0, q1, [x9, #32]
	prfm	PLDL1KEEP, [x9, x15]
	add		x9, x9, x10
	stp		q0, q1, [x13, #96]

	sub		w8, w8, #4
	add		x11, x11, #128
	add		x13, x13, #128

	cmp		w8, #3
	bgt		1b


	cmp		w8, #0
	ble		0f


2: // clean-up loop
	ldp		q0, q1, [x9, #0]
	stp		q0, q1, [x11, #0]
	ldp		q0, q1, [x9, #32]
	add		x9, x9, x10
	stp		q0, q1, [x13, #0]

	sub		w8, w8, #1
	add		x11, x11, #32
	add		x13, x13, #32

	cmp		w8, #0
	bgt		2b


0: // return

	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dpack_nn_8_lib4)





//                             w0        x1         w2       x3
// void kernel_dpack_nn_4_lib4(int kmax, double *A, int lda, double *C)

	.align	4
	GLOB(kernel_dpack_nn_4_lib4)
	FUN_START(kernel_dpack_nn_4_lib4)
	


	PROLOGUE



	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // lda
	lsl		w10, w10, #3 // 8*lda
	mov		x11, x3 // C


	cmp		w8, #0
	ble		0f

	lsl		w12, w10, #2 // 4*(lda*sizeof(double))

	cmp		w8, #3
	ble		2f


1: // main loop
	ldp		q0, q1, [x9, #0]
	prfm	PLDL1KEEP, [x9, x12]
	add		x9, x9, x10
	stp		q0, q1, [x11, #0]

	ldp		q0, q1, [x9, #0]
	prfm	PLDL1KEEP, [x9, x12]
	add		x9, x9, x10
	stp		q0, q1, [x11, #32]

	ldp		q0, q1, [x9, #0]
	prfm	PLDL1KEEP, [x9, x12]
	add		x9, x9, x10
	stp		q0, q1, [x11, #64]

	ldp		q0, q1, [x9, #0]
	prfm	PLDL1KEEP, [x9, x12]
	add		x9, x9, x10
	stp		q0, q1, [x11, #96]

	sub		w8, w8, #4
	add		x11, x11, #128

	cmp		w8, #3
	bgt		1b


	cmp		w8, #0
	ble		0f


2: // clean-up loop
	ldp		q0, q1, [x9, #0]
	add		x9, x9, x10
	stp		q0, q1, [x11, #0]

	sub		w8, w8, #1
	add		x11, x11, #32

	cmp		w8, #0
	bgt		2b


0: // return

	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dpack_nn_4_lib4)





//                             w0        x1         w2       x3
// void kernel_dpack_tn_4_lib4(int kmax, double *A, int lda, double *C)

	.align	4
	GLOB(kernel_dpack_tn_4_lib4)
	FUN_START(kernel_dpack_tn_4_lib4)
	


	PROLOGUE



	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // lda
	lsl		w10, w10, #3 // 8*sda
	mov		x11, x3 // C


	cmp		w8, #0
	ble		0f

	cmp		w8, #3
	ble		2f


1: // main loop
	mov		x13, x9

	ldp		q0, q1, [x13, #0]
	prfm	PLDL1KEEP, [x13, #64]
	add		x13, x13, x10
	ldp		q2, q3, [x13, #0]
	prfm	PLDL1KEEP, [x13, #64]
	add		x13, x13, x10
	ldp		q4, q5, [x13, #0]
	prfm	PLDL1KEEP, [x13, #64]
	add		x13, x13, x10
	ldp		q6, q7, [x13, #0]
	prfm	PLDL1KEEP, [x13, #64]
	add		x13, x13, x10

	prfm	PLDL1KEEP, [x11, #(128+0)]
	prfm	PLDL1KEEP, [x11, #(128+64)]

	trn1	v24.2d, v0.2d, v2.2d
	trn2	v2.2d, v0.2d, v2.2d
	trn1	v25.2d, v5.2d, v7.2d
	trn2	v7.2d, v5.2d, v7.2d
	trn1	v26.2d, v1.2d, v3.2d
	trn2	v27.2d, v1.2d, v3.2d
	trn1	v1.2d, v4.2d, v6.2d
	trn2	v3.2d, v4.2d, v6.2d

//	mov		v0.16b, v24.16b
//	mov		v5.16b, v25.16b
//	mov		v4.16b, v26.16b
//	mov		v6.16b, v27.16b

//	stp		q0, q1, [x11, #0]
	stp		q24, q1, [x11, #0]
	stp		q2, q3, [x11, #32]
//	stp		q4, q5, [x11, #64]
	stp		q26, q25, [x11, #64]
//	stp		q6, q7, [x11, #96]
	stp		q27, q7, [x11, #96]

	sub		w8, w8, #4
	add		x9, x9, #32
	add		x11, x11, #128

	cmp		w8, #3
	bgt		1b


	cmp		w8, #0
	ble		0f


2: // clean-up loop
	mov		x13, x9

	ldr		d0, [x13, #0]
	add		x13, x13, x10
	str		d0, [x11, #0]

	ldr		d0, [x13, #0]
	add		x13, x13, x10
	str		d0, [x11, #8]

	ldr		d0, [x13, #0]
	add		x13, x13, x10
	str		d0, [x11, #16]

	ldr		d0, [x13, #0]
	add		x13, x13, x10
	str		d0, [x11, #24]

	sub		w8, w8, #1
	add		x9, x9, #8
	add		x11, x11, #32

	cmp		w8, #0
	bgt		2b


0: // return

	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dpack_tn_4_lib4)





//                              w0        x1         w2       x3         w4
// void kernel_dpack_tt_12_lib4(int kmax, double *A, int lda, double *C, ind sdc)

	.align	4
	GLOB(kernel_dpack_tt_12_lib4)
	FUN_START(kernel_dpack_tt_12_lib4)
	


	PROLOGUE



	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // A
	lsl		w10, w10, #3 // 32*sda
	mov		x11, x3 // A
	mov		w12, w4 // sdc
	lsl		w12, w12, #5 // 32*sdc


	cmp		w8, #0
	ble		0f

	cmp		w8, #3
	ble		2f


1: // main loop
	mov		x13, x9

	// TODO prefetch

	ldp		q0, q1, [x13, #0]
	add		x13, x13, x10
	stp		q0, q1, [x11, #0]

	ldp		q0, q1, [x13, #0]
	add		x13, x13, x10
	stp		q0, q1, [x11, #32]

	ldp		q0, q1, [x13, #0]
	add		x13, x13, x10
	stp		q0, q1, [x11, #64]

	ldp		q0, q1, [x13, #0]
	add		x13, x13, x10
	stp		q0, q1, [x11, #96]

	ldp		q0, q1, [x13, #0]
	add		x13, x13, x10
	stp		q0, q1, [x11, #128]

	ldp		q0, q1, [x13, #0]
	add		x13, x13, x10
	stp		q0, q1, [x11, #160]

	ldp		q0, q1, [x13, #0]
	add		x13, x13, x10
	stp		q0, q1, [x11, #192]

	ldp		q0, q1, [x13, #0]
	add		x13, x13, x10
	stp		q0, q1, [x11, #224]

	ldp		q0, q1, [x13, #0]
	add		x13, x13, x10
	stp		q0, q1, [x11, #256]

	ldp		q0, q1, [x13, #0]
	add		x13, x13, x10
	stp		q0, q1, [x11, #288]

	ldp		q0, q1, [x13, #0]
	add		x13, x13, x10
	stp		q0, q1, [x11, #320]

	ldp		q0, q1, [x13, #0]
	add		x13, x13, x10
	stp		q0, q1, [x11, #352]

	sub		w8, w8, #4
	add		x9, x9, #32
	add		x11, x11, x12

	cmp		w8, #3
	bgt		1b


	cmp		w8, #0
	ble		0f


2: // clean-up loop
	mov		x13, x9

	ldr		d0, [x13, #0]
	add		x13, x13, x10
	str		d0, [x11, #0]

	ldr		d0, [x13, #0]
	add		x13, x13, x10
	str		d0, [x11, #32]

	ldr		d0, [x13, #0]
	add		x13, x13, x10
	str		d0, [x11, #64]

	ldr		d0, [x13, #0]
	add		x13, x13, x10
	str		d0, [x11, #96]

	ldr		d0, [x13, #0]
	add		x13, x13, x10
	str		d0, [x11, #128]

	ldr		d0, [x13, #0]
	add		x13, x13, x10
	str		d0, [x11, #160]

	ldr		d0, [x13, #0]
	add		x13, x13, x10
	str		d0, [x11, #192]

	ldr		d0, [x13, #0]
	add		x13, x13, x10
	str		d0, [x11, #224]

	ldr		d0, [x13, #0]
	add		x13, x13, x10
	str		d0, [x11, #256]

	ldr		d0, [x13, #0]
	add		x13, x13, x10
	str		d0, [x11, #288]

	ldr		d0, [x13, #0]
	add		x13, x13, x10
	str		d0, [x11, #320]

	ldr		d0, [x13, #0]
	add		x13, x13, x10
	str		d0, [x11, #352]

	sub		w8, w8, #1
	add		x9, x9, #8
	add		x11, x11, #8

	cmp		w8, #0
	bgt		2b


0: // return

	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dpack_tt_12_lib4)





//                             w0        x1         w2       x3         w4
// void kernel_dpack_tt_8_lib4(int kmax, double *A, int lda, double *C, ind sdc)

	.align	4
	GLOB(kernel_dpack_tt_8_lib4)
	FUN_START(kernel_dpack_tt_8_lib4)
	


	PROLOGUE



	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // A
	lsl		w10, w10, #3 // 32*sda
	mov		x11, x3 // A
	mov		w12, w4 // sdc
	lsl		w12, w12, #5 // 32*sdc


	cmp		w8, #0
	ble		0f

	cmp		w8, #3
	ble		2f


1: // main loop
	mov		x13, x9

	// TODO prefetch

	ldp		q0, q1, [x13, #0]
	add		x13, x13, x10
	stp		q0, q1, [x11, #0]

	ldp		q0, q1, [x13, #0]
	add		x13, x13, x10
	stp		q0, q1, [x11, #32]

	ldp		q0, q1, [x13, #0]
	add		x13, x13, x10
	stp		q0, q1, [x11, #64]

	ldp		q0, q1, [x13, #0]
	add		x13, x13, x10
	stp		q0, q1, [x11, #96]

	ldp		q0, q1, [x13, #0]
	add		x13, x13, x10
	stp		q0, q1, [x11, #128]

	ldp		q0, q1, [x13, #0]
	add		x13, x13, x10
	stp		q0, q1, [x11, #160]

	ldp		q0, q1, [x13, #0]
	add		x13, x13, x10
	stp		q0, q1, [x11, #192]

	ldp		q0, q1, [x13, #0]
	add		x13, x13, x10
	stp		q0, q1, [x11, #224]

	sub		w8, w8, #4
	add		x9, x9, #32
	add		x11, x11, x12

	cmp		w8, #3
	bgt		1b


	cmp		w8, #0
	ble		0f


2: // clean-up loop
	mov		x13, x9

	ldr		d0, [x13, #0]
	add		x13, x13, x10
	str		d0, [x11, #0]

	ldr		d0, [x13, #0]
	add		x13, x13, x10
	str		d0, [x11, #32]

	ldr		d0, [x13, #0]
	add		x13, x13, x10
	str		d0, [x11, #64]

	ldr		d0, [x13, #0]
	add		x13, x13, x10
	str		d0, [x11, #96]

	ldr		d0, [x13, #0]
	add		x13, x13, x10
	str		d0, [x11, #128]

	ldr		d0, [x13, #0]
	add		x13, x13, x10
	str		d0, [x11, #160]

	ldr		d0, [x13, #0]
	add		x13, x13, x10
	str		d0, [x11, #192]

	ldr		d0, [x13, #0]
	add		x13, x13, x10
	str		d0, [x11, #224]

	sub		w8, w8, #1
	add		x9, x9, #8
	add		x11, x11, #8

	cmp		w8, #0
	bgt		2b


0: // return

	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dpack_tt_8_lib4)





//                             w0        x1         w2       x3         w4
// void kernel_dpack_tt_4_lib4(int kmax, double *A, int lda, double *C, ind sdc)

	.align	4
	GLOB(kernel_dpack_tt_4_lib4)
	FUN_START(kernel_dpack_tt_4_lib4)
	


	PROLOGUE



	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // lda
	lsl		w10, w10, #3 // 8*lda
	mov		x11, x3 // C
	mov		w12, w4 // sdc
	lsl		w12, w12, #5 // 32*sdc


	cmp		w8, #0
	ble		0f

	cmp		w8, #3
	ble		2f


1: // main loop
	mov		x13, x9
	add		x14, x11, x12

	// TODO prefetch

	ldp		q0, q1, [x13, #0]
	prfm	PLDL1KEEP, [x13, #64]
	add		x13, x13, x10
	stp		q0, q1, [x11, #0]
	prfm	PLDL1KEEP, [x14, #0]

	ldp		q0, q1, [x13, #0]
	prfm	PLDL1KEEP, [x13, #64]
	add		x13, x13, x10
	stp		q0, q1, [x11, #32]
	prfm	PLDL1KEEP, [x14, #32]

	ldp		q0, q1, [x13, #0]
	prfm	PLDL1KEEP, [x13, #64]
	add		x13, x13, x10
	stp		q0, q1, [x11, #64]
	prfm	PLDL1KEEP, [x14, #64]

	ldp		q0, q1, [x13, #0]
	prfm	PLDL1KEEP, [x13, #64]
	add		x13, x13, x10
	stp		q0, q1, [x11, #96]
	prfm	PLDL1KEEP, [x14, #96]

	sub		w8, w8, #4
	add		x9, x9, #32
//	add		x11, x11, x12
	mov		x11, x14

	cmp		w8, #3
	bgt		1b


	cmp		w8, #0
	ble		0f


2: // clean-up loop
	mov		x13, x9

	ldr		d0, [x13, #0]
	add		x13, x13, x10
	str		d0, [x11, #0]

	ldr		d0, [x13, #0]
	add		x13, x13, x10
	str		d0, [x11, #32]

	ldr		d0, [x13, #0]
	add		x13, x13, x10
	str		d0, [x11, #64]

	ldr		d0, [x13, #0]
	add		x13, x13, x10
	str		d0, [x11, #96]

	sub		w8, w8, #1
	add		x9, x9, #8
	add		x11, x11, #8

	cmp		w8, #0
	bgt		2b


0: // return

	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dpack_tt_4_lib4)





//                                w0        x1         w2       x3         w4
// void kernel_dunpack_nn_12_lib4(int kmax, double *A, int sda, double *C, int ldc)

	.align	4
	GLOB(kernel_dunpack_nn_12_lib4)
	FUN_START(kernel_dunpack_nn_12_lib4)
	


	PROLOGUE



	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x3 // C
	mov		w12, w4 // ldc
	lsl		w12, w12, #3 // 8*ldc


	cmp		w8, #0
	ble		0f

	add		x13, x9, x10
	add		x14, x13, x10

	cmp		w8, #3
	ble		2f


1: // main loop
	ldp		q0, q1, [x9, #0]
	stp		q0, q1, [x11, #0]
	ldp		q0, q1, [x13, #0]
	stp		q0, q1, [x11, #32]
	ldp		q0, q1, [x14, #0]
	stp		q0, q1, [x11, #64]
	add		x11, x11, x12

	ldp		q0, q1, [x9, #32]
	stp		q0, q1, [x11, #0]
	ldp		q0, q1, [x13, #32]
	stp		q0, q1, [x11, #32]
	ldp		q0, q1, [x14, #32]
	stp		q0, q1, [x11, #64]
	add		x11, x11, x12

	ldp		q0, q1, [x9, #64]
	stp		q0, q1, [x11, #0]
	ldp		q0, q1, [x13, #64]
	stp		q0, q1, [x11, #32]
	ldp		q0, q1, [x14, #64]
	stp		q0, q1, [x11, #64]
	add		x11, x11, x12

	ldp		q0, q1, [x9, #96]
	stp		q0, q1, [x11, #0]
	ldp		q0, q1, [x13, #96]
	stp		q0, q1, [x11, #32]
	ldp		q0, q1, [x14, #96]
	stp		q0, q1, [x11, #64]
	add		x11, x11, x12

	sub		w8, w8, #4
	add		x9, x9, #128
	add		x13, x13, #128
	add		x14, x14, #128

	cmp		w8, #3
	bgt		1b


	cmp		w8, #0
	ble		0f


2: // clean-up loop
	ldp		q0, q1, [x9, #0]
	stp		q0, q1, [x11, #0]
	ldp		q0, q1, [x13, #0]
	stp		q0, q1, [x11, #32]
	ldp		q0, q1, [x14, #0]
	stp		q0, q1, [x11, #64]
	add		x11, x11, x12

	sub		w8, w8, #1
	add		x9, x9, #32
	add		x13, x13, #32
	add		x14, x14, #32

	cmp		w8, #0
	bgt		2b


0: // return

	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dunpack_nn_12_lib4)





//                               w0        x1         w2       x3         w4
// void kernel_dunpack_nn_8_lib4(int kmax, double *A, int sda, double *C, int ldc)

	.align	4
	GLOB(kernel_dunpack_nn_8_lib4)
	FUN_START(kernel_dunpack_nn_8_lib4)
	


	PROLOGUE



	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #5 // 32*sda
	mov		x11, x3 // C
	mov		w12, w4 // ldc
	lsl		w12, w12, #3 // 8*ldc


	cmp		w8, #0
	ble		0f

	add		x13, x9, x10

	cmp		w8, #3
	ble		2f


1: // main loop
	ldp		q0, q1, [x9, #0]
	stp		q0, q1, [x11, #0]
	ldp		q0, q1, [x13, #0]
	stp		q0, q1, [x11, #32]
	add		x11, x11, x12

	ldp		q0, q1, [x9, #32]
	stp		q0, q1, [x11, #0]
	ldp		q0, q1, [x13, #32]
	stp		q0, q1, [x11, #32]
	add		x11, x11, x12

	ldp		q0, q1, [x9, #64]
	stp		q0, q1, [x11, #0]
	ldp		q0, q1, [x13, #64]
	stp		q0, q1, [x11, #32]
	add		x11, x11, x12

	ldp		q0, q1, [x9, #96]
	stp		q0, q1, [x11, #0]
	ldp		q0, q1, [x13, #96]
	stp		q0, q1, [x11, #32]
	add		x11, x11, x12

	sub		w8, w8, #4
	add		x9, x9, #128
	add		x13, x13, #128

	cmp		w8, #3
	bgt		1b


	cmp		w8, #0
	ble		0f


2: // clean-up loop
	ldp		q0, q1, [x9, #0]
	stp		q0, q1, [x11, #0]
	ldp		q0, q1, [x13, #0]
	stp		q0, q1, [x11, #32]
	add		x11, x11, x12

	sub		w8, w8, #1
	add		x9, x9, #32
	add		x13, x13, #32

	cmp		w8, #0
	bgt		2b


0: // return

	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dunpack_nn_8_lib4)





//                               w0        x1         x2         w3
// void kernel_dunpack_nn_4_lib4(int kmax, double *A, double *C, int ldc)

	.align	4
	GLOB(kernel_dunpack_nn_4_lib4)
	FUN_START(kernel_dunpack_nn_4_lib4)
	


	PROLOGUE



	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		x10, x2 // C
	mov		w11, w3 // ldc
	lsl		w11, w11, #3 // 8*ldc


	cmp		w8, #0
	ble		0f

	cmp		w8, #3
	ble		2f


1: // main loop
	ldp		q0, q1, [x9, #0]
	stp		q0, q1, [x10, #0]
	add		x10, x10, x11

	ldp		q0, q1, [x9, #32]
	stp		q0, q1, [x10, #0]
	add		x10, x10, x11

	ldp		q0, q1, [x9, #64]
	stp		q0, q1, [x10, #0]
	add		x10, x10, x11

	ldp		q0, q1, [x9, #96]
	stp		q0, q1, [x10, #0]
	add		x10, x10, x11

	sub		w8, w8, #4
	add		x9, x9, #128

	cmp		w8, #3
	bgt		1b


	cmp		w8, #0
	ble		0f


2: // clean-up loop
	ldp		q0, q1, [x9, #0]
	stp		q0, q1, [x10, #0]
	add		x10, x10, x11

	sub		w8, w8, #1
	add		x9, x9, #32

	cmp		w8, #0
	bgt		2b


0: // return

	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dunpack_nn_4_lib4)






