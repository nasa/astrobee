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





//                             w0        x1         w2       x3         w4
// void kernel_spack_nn_8_lib4(int kmax, double *A, int lda, double *C, int sdc)

	.align	4
	GLOB(kernel_spack_nn_8_lib4)
	FUN_START(kernel_spack_nn_8_lib4)
	


	PROLOGUE



	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // lda
	lsl		w10, w10, #2 // 4*lda
	mov		x11, x3 // C
	mov		w12, w4 // sdc
	lsl		w12, w12, #4 // 16*sdc


	cmp		w8, #0
	ble		0f

	add		x13, x11, x12

//	add		x14, x10, x10
//	add		x14, x14, x14
////	add		x14, x14, x14
	lsl		x14, x10, #2 // 4*(...)

	cmp		w8, #3
	ble		2f


1: // main loop
	ldp		q0, q1, [x9, #0]
	prfm	PLDL1KEEP, [x9, x14]
	add		x9, x9, x10
	ldp		q2, q3, [x9, #0]
	prfm	PLDL1KEEP, [x9, x14]
	add		x9, x9, x10
	stp		q0, q2, [x11, #0]
	stp		q1, q3, [x13, #0]

	ldp		q0, q1, [x9, #0]
	prfm	PLDL1KEEP, [x9, x14]
	add		x9, x9, x10
	ldp		q2, q3, [x9, #0]
	prfm	PLDL1KEEP, [x9, x14]
	add		x9, x9, x10
	stp		q0, q2, [x11, #32]
	stp		q1, q3, [x13, #32]

	sub		w8, w8, #4
	add		x11, x11, #64
	add		x13, x13, #64

	cmp		w8, #3
	bgt		1b


	cmp		w8, #0
	ble		0f


2: // clean-up loop
	ldp		q0, q1, [x9, #0]
	add		x9, x9, x10
	str		q0, [x11, #0]
	str		q1, [x13, #0]

	sub		w8, w8, #1
	add		x11, x11, #16
	add		x13, x13, #16

	cmp		w8, #0
	bgt		2b


0: // return

	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_spack_nn_8_lib4)





//                             w0        x1         w2       x3
// void kernel_spack_nn_4_lib4(int kmax, double *A, int lda, double *C)

	.align	4
	GLOB(kernel_spack_nn_4_lib4)
	FUN_START(kernel_spack_nn_4_lib4)
	


	PROLOGUE



	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // lda
	lsl		w10, w10, #2 // 4*lda
	mov		x11, x3 // C


	cmp		w8, #0
	ble		0f

//	add		x14, x10, x10
//	add		x14, x14, x14
////	add		x14, x14, x14
	lsl		x14, x10, #2 // 4*(...)

	cmp		w8, #3
	ble		2f


1: // main loop
	ldr		q0, [x9, #0]
	prfm	PLDL1KEEP, [x9, x14]
	add		x9, x9, x10
	str		q0, [x11, #0]

	ldr		q0, [x9, #0]
	prfm	PLDL1KEEP, [x9, x14]
	add		x9, x9, x10
	str		q0, [x11, #16]

	ldr		q0, [x9, #0]
	prfm	PLDL1KEEP, [x9, x14]
	add		x9, x9, x10
	str		q0, [x11, #32]

	ldr		q0, [x9, #0]
	prfm	PLDL1KEEP, [x9, x14]
	add		x9, x9, x10
	str		q0, [x11, #48]

	sub		w8, w8, #4
	add		x11, x11, #64

	cmp		w8, #3
	bgt		1b


	cmp		w8, #0
	ble		0f


2: // clean-up loop
	ldr		q0, [x9, #0]
	add		x9, x9, x10
	str		q0, [x11, #0]

	sub		w8, w8, #1
	add		x11, x11, #16

	cmp		w8, #0
	bgt		2b


0: // return

	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_spack_nn_4_lib4)





//                             w0        x1         w2       x3
// void kernel_spack_tn_4_lib4(int kmax, double *A, int lda, double *C)

	.align	4
	GLOB(kernel_spack_tn_4_lib4)
	FUN_START(kernel_spack_tn_4_lib4)
	


	PROLOGUE



	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // lda
	lsl		w10, w10, #2 // 4*lda
	mov		x11, x3 // C


	cmp		w8, #0
	ble		0f

	cmp		w8, #3
	ble		2f


1: // main loop
	mov		x13, x9

	ldr		q0, [x13, #0]
	add		x13, x13, x10
	ldr		q1, [x13, #0]
	add		x13, x13, x10
	ldr		q2, [x13, #0]
	add		x13, x13, x10
	ldr		q3, [x13, #0]
	add		x13, x13, x10

	trn1	v4.4s, v0.4s, v1.4s
	trn2	v5.4s, v0.4s, v1.4s
	trn1	v6.4s, v2.4s, v3.4s
	trn2	v7.4s, v2.4s, v3.4s

	trn1	v0.2d, v4.2d, v6.2d
	trn2	v2.2d, v4.2d, v6.2d
	trn1	v1.2d, v5.2d, v7.2d
	trn2	v3.2d, v5.2d, v7.2d

	str		q0, [x11, #0]
	str		q1, [x11, #16]
	str		q2, [x11, #32]
	str		q3, [x11, #48]

	sub		w8, w8, #4
	add		x9, x9, #16
	add		x11, x11, #64

	cmp		w8, #3
	bgt		1b


	cmp		w8, #0
	ble		0f


2: // clean-up loop
	mov		x13, x9

	ldr		s0, [x13, #0]
	add		x13, x13, x10
	str		s0, [x11, #0]

	ldr		s0, [x13, #0]
	add		x13, x13, x10
	str		s0, [x11, #4]

	ldr		s0, [x13, #0]
	add		x13, x13, x10
	str		s0, [x11, #8]

	ldr		s0, [x13, #0]
	add		x13, x13, x10
	str		s0, [x11, #12]

	sub		w8, w8, #1
	add		x9, x9, #4
	add		x11, x11, #16

	cmp		w8, #0
	bgt		2b


0: // return

	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_spack_tn_4_lib4)





//                             w0        x1         w2       x3         w4
// void kernel_spack_tt_4_lib4(int kmax, double *A, int lda, double *C, ind sdc)

	.align	4
	GLOB(kernel_spack_tt_4_lib4)
	FUN_START(kernel_spack_tt_4_lib4)
	


	PROLOGUE



	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // lda
	lsl		w10, w10, #2 // 4*lda
	mov		x11, x3 // C
	mov		w12, w4 // sdc
	lsl		w12, w12, #4 // 16*sdc


	cmp		w8, #0
	ble		0f

	cmp		w8, #3
	ble		2f


1: // main loop
	mov		x13, x9

	ldr		q0, [x13, #0]
	add		x13, x13, x10
	str		q0, [x11, #0]

	ldr		q0, [x13, #0]
	add		x13, x13, x10
	str		q0, [x11, #16]

	ldr		q0, [x13, #0]
	add		x13, x13, x10
	str		q0, [x11, #32]

	ldr		q0, [x13, #0]
	add		x13, x13, x10
	str		q0, [x11, #48]

	sub		w8, w8, #4
	add		x9, x9, #16
	add		x11, x11, x12

	cmp		w8, #3
	bgt		1b


	cmp		w8, #0
	ble		0f


2: // clean-up loop
	mov		x13, x9

	ldr		s0, [x13, #0]
	add		x13, x13, x10
	str		s0, [x11, #0]

	ldr		s0, [x13, #0]
	add		x13, x13, x10
	str		s0, [x11, #16]

	ldr		s0, [x13, #0]
	add		x13, x13, x10
	str		s0, [x11, #32]

	ldr		s0, [x13, #0]
	add		x13, x13, x10
	str		s0, [x11, #48]

	sub		w8, w8, #1
	add		x9, x9, #4
	add		x11, x11, #4

	cmp		w8, #0
	bgt		2b


0: // return

	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_spack_tt_4_lib4)






