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
#define ZERO_ACC_N \
	fmov	d0, xzr
#define ZERO_ACC_T \
	fmov	d0, xzr; \
	fmov    d1, d0; \
	fmov    d2, d0; \
	fmov    d3, d0

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
.macro ZERO_ACC_N
	fmov	d0, xzr
.endm
.macro ZERO_ACC_T
	fmov	d0, xzr
	fmov    d1, d0
	fmov    d2, d0
	fmov    d3, d0
.endm

#endif






	.text





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- x
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMV_N_4_LIB4
#else
	.align	4
	FUN_START(inner_kernel_gemv_n_4_lib4)
#endif

	// early return
	cmp		w8, #0
	ble		2f // return

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc
	fmov	d8, xzr
	fmov    d9, d8
	fmov    d10, d8

	// main loop
1:
	
	ldr		q24, [x10, #0]

	ldp		q16, q17, [x9, #(0*32)]
	ldp		q18, q19, [x9, #(1*32)]

	// unroll 0
	fmla	v0.4s, v16.4s, v24.s[0]
	add		x10, x10, #16

	// unroll 1
	fmla	v8.4s, v17.4s, v24.s[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v9.4s, v18.4s, v24.s[2]
	cmp		w8, #4

	// unroll 3
	fmla	v10.4s, v19.4s, v24.s[3]
	add		x9, x9, #64

	bgt		1b


	// reduce
	fadd	v0.4s, v0.4s, v8.4s
	fadd	v9.4s, v9.4s, v10.4s
	fadd	v0.4s, v0.4s, v9.4s

0:

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		s24, [x10]
	ld1		{v16.4s}, [x9], #16
	fmla	v0.4s, v16.4s, v24.s[0]
	add		x10, x10, #4

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemv_n_4_lib4)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// w10  <- 16*sda
// x11  <- x
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMV_T_4_LIB4
#else
	.align	4
	FUN_START(inner_kernel_gemv_t_4_lib4)
#endif

	// early return
	cmp		w8, #0
	ble		2f // return

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #0]

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	add		x12, x10, #64

	// prefetch
	prfm	PLDL1KEEP, [x9, x10]
//	prfm	PLDL1KEEP, [x11, #16]

	// main loop
1:
	
	ldr		q24, [x11, #(0*32)]

	ldp		q16, q17, [x9, #(0*32)]
	ldp		q18, q19, [x9, #(1*32)]

	// unroll x4
	fmla	v0.4s, v16.4s, v24.4s
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, x10]

	fmla	v1.4s, v17.4s, v24.4s
	add		x11, x11, #16

	fmla	v2.4s, v18.4s, v24.4s
//	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x11, #16]
	sub		w8, w8, #4

	fmla	v3.4s, v19.4s, v24.4s
	cmp		w8, #4

	bgt		1b

0:

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		s24, [x11], #4
	ldr		s16, [x9, #(0*16)]
	ldr		s18, [x9, #(1*16)]
	ldr		s20, [x9, #(2*16)]
	ldr		s22, [x9, #(3*16)]
	fmla	v0.4s, v16.4s, v24.4s
	fmla	v1.4s, v18.4s, v24.4s
	fmla	v2.4s, v20.4s, v24.4s
	fmla	v3.4s, v22.4s, v24.4s
	add		x9, x9, #4

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemv_t_4_lib4)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// w10  <- 16*sda
// x11  <- x
// w12  <- offA
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_GEMV_T_4_LIB4
#else
	.align	4
	FUN_START(inner_edge_gemv_t_4_lib4)
#endif

	cmp		w12, #0
	ble		2f // return

	cmp		w8, #0
	ble		2f // return

	mov		w13, #4
	sub		w14, w13, w12 // 4-offsetB
	cmp		w14, w8
	ble		0f
	mov		w14, w8 // kend=min(k,4-offsetB)
0:
//	movgt	w14, w8 // kend=min(k,4-offsetB)
	
//	add		x9, x9, x12, LSL #2 // A + offA*sizeof(float)

1:
	ldr		s24, [x11], #4
	ldr		s16, [x9, #(0*16)]
	ldr		s18, [x9, #(1*16)]
	ldr		s20, [x9, #(2*16)]
	ldr		s22, [x9, #(3*16)]
	fmla	v0.4s, v16.4s, v24.4s
	fmla	v1.4s, v18.4s, v24.4s
	fmla	v2.4s, v20.4s, v24.4s
	fmla	v3.4s, v22.4s, v24.4s
	add		x9, x9, #4
	sub		w8, w8, #1

	sub		w14, w14, #1

	cmp		w14, #0
	bgt		1b

	cmp		w8, #0
	ble		2f // return

	add		x9, x9, x10
	sub		x9, x9, #16

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_gemv_t_4_lib4)
#endif





// subroutine
//
// triangular substitution with vector RHS
//
// input arguments:
// x8  <- E
// x9  <- inv_diag_E
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRSV_LN_INV_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_trsv_ln_inv_4_lib4)
#endif
	
	ldr		q24, [x8, #0]
	ldr		s16, [x9, #0]
	fmul	v17.4s, v0.4s, v16.s[0]
	fmls	v0.4s, v24.4s, v17.s[0]

	ldr		q24, [x8, #16]
	ldr		s16, [x9, #4]
	fmul	v18.4s, v0.4s, v16.s[0]
	fmls	v0.4s, v24.4s, v18.s[1]

	ldr		q24, [x8, #32]
	ldr		s16, [x9, #8]
	fmul	v19.4s, v0.4s, v16.s[0]
	fmls	v0.4s, v24.4s, v19.s[2]

	ldr		s16, [x9, #12]
//	fmul	v20.4s, v0.4s, v16.s[0]
	fmul	v0.4s, v0.4s, v16.s[0]

//	ins		v0.s[3], v20.s[3]
	ins		v0.s[2], v19.s[2]
	ins		v0.s[1], v18.s[1]
	ins		v0.s[0], v17.s[0]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsv_ln_inv_4_lib4)
#endif





// subroutine
//
// triangular substitution with vector RHS
//
// input arguments:
// x8  <- E
// x9  <- inv_diag_E
// w10 <- kn
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRSV_LN_INV_4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_trsv_ln_inv_4_vs_lib4)
#endif
	
	ldr		q24, [x8, #0]
	ldr		s16, [x9, #0]
	fmul	v17.4s, v0.4s, v16.s[0]
	fmls	v0.4s, v24.4s, v17.s[0]

	cmp		w10, #2
	blt		1f // return

	ldr		q24, [x8, #16]
	ldr		s16, [x9, #4]
	fmul	v18.4s, v0.4s, v16.s[0]
	fmls	v0.4s, v24.4s, v18.s[1]

	cmp		w10, #3
	blt		2f // return

	ldr		q24, [x8, #32]
	ldr		s16, [x9, #8]
	fmul	v19.4s, v0.4s, v16.s[0]
	fmls	v0.4s, v24.4s, v19.s[2]

	cmp		w10, #4
	blt		3f // return

	ldr		s16, [x9, #12]
//	fmul	v20.4s, v0.4s, v16.s[0]
	fmul	v0.4s, v0.4s, v16.s[0]

//	ins		v0.s[3], v20.s[3]
3:
	ins		v0.s[2], v19.s[2]
2:
	ins		v0.s[1], v18.s[1]
1:
	ins		v0.s[0], v17.s[0]

0:
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsv_ln_inv_4_vs_lib4)
#endif





#if 0
// TODO
// subroutine
//
// triangular substitution with vector RHS
//
// input arguments:
// x8  <- E
// x9  <- inv_diag_E
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRSV_LT_INV_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_trsv_lt_inv_4_lib4)
#endif

	ldr		d16, [x9, #24]
	fmul	v20.2d, v1.2d, v16.2d[0]

	ldr		d25, [x8, #(2*32+24)]
	ldr		d16, [x9, #16]
	fmls	v1.2d, v25.2d, v20.2d[1]
	fmul	v1.2d, v1.2d, v16.2d[0]
	ins		v1.d[1], v20.d[1]

	ldr		q24, [x8, #(0*32+16)]
	ldr		q25, [x8, #(1*32+16)]
	trn1	v26.2d, v24.2d, v25.2d
	trn2	v27.2d, v24.2d, v25.2d
	fmls	v0.2d, v26.2d, v1.2d[0]
	fmls	v0.2d, v27.2d, v1.2d[1]

	ldr		d16, [x9, #8]
	fmul	v20.2d, v0.2d, v16.2d[0]

	ldr		d25, [x8, #(0+8)]
	ldr		d16, [x9, #0]
	fmls	v0.2d, v25.2d, v20.2d[1]
	fmul	v0.2d, v0.2d, v16.2d[0]
	ins		v0.d[1], v20.d[1]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsv_lt_inv_4_lib4)
#endif
#endif





// subroutine
//
// input arguments:
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_T_4_LIB4
#else
	.align	4
	FUN_START(inner_blend_t_4_lib4)
#endif

	faddp	v0.4s, v0.4s, v1.4s
	faddp	v2.4s, v2.4s, v3.4s
	faddp	v0.4s, v0.4s, v2.4s

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_blend_t_4_lib4)
#endif





// subroutine
//
// input arguments:
// x8   <- alpha
// x9   <- beta
// x10  <- y
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_4_LIB4
#else
	.align	4
	FUN_START(inner_scale_ab_4_lib4)
#endif

	ldr		s28, [x8]

	ldr		s29, [x9]

	fmul	v0.4s, v0.4s, v28.s[0]

	fcmpe	s29, #0.0
	beq		0f

	ldr		q24, [x10, #0]
	fmla	v0.4s, v24.4s, v29.s[0]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_4_lib4)
#endif





// subroutine
//
// input arguments:
// x8  <- y
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M11_4_LIB4
#else
	.align	4
	FUN_START(inner_scale_m11_4_lib4)
#endif

	ldr		q24, [x8, #0]
	fsub	v0.4s, v24.4s, v0.4s

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m11_4_lib4)
#endif





// subroutine
//
// input arguments:
// x8   <- z
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4_LIB4
#else
	.align 4
	FUN_START(inner_store_4_lib4)
#endif

	str		q0, [x8, #0]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4_lib4)
#endif





// subroutine
//
// input arguments:
// x8   <- z
// x9   <- m1
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4_VS_LIB4
#else
	.align 4
	FUN_START(inner_store_4_vs_lib4)
#endif

	cmp		w9, #4
	bge		1f
	cmp		w9, #2
	bgt		2f
	beq		3f

	// 1
	str		s0, [x8, #0]
	b		0f

3:
	// 2
	str		d0, [x8, #0]
	b		0f

2:
	// 3
	ins		v1.s[0], v0.s[2]
	str		d0, [x8, #0]
	str		s1, [x8, #8]
	b		0f

1:
	// 4
	str		q0, [x8, #0]
	b		0f

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4_vs_lib4)
#endif





//                            w0        x1             x2         x3         x4            x5         x6
// void kernel_sgemv_n_4_lib4(int kmax, double *alpha, double *A, double *x, double *beta, double *y, double *z)

	.align	4
	GLOB(kernel_sgemv_n_4_lib4)
	FUN_START(kernel_sgemv_n_4_lib4)
	

	PROLOGUE


	ZERO_ACC_N


	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_N_4_LIB4
#else
	CALL(inner_kernel_gemv_n_4_lib4)
#endif


	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x4 // beta
	mov		x10, x5 // y

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4_LIB4
#else
	CALL(inner_scale_ab_4_lib4)
#endif


	// store n
	mov		x8, x6 // z

#if MACRO_LEVEL>=1
	INNER_STORE_4_LIB4
#else
	CALL(inner_store_4_lib4)
#endif


	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemv_n_4_lib4)





//                               w0        x1             x2         x3         x4            x5         x6         w7
// void kernel_sgemv_n_4_vs_lib4(int kmax, double *alpha, double *A, double *x, double *beta, double *y, double *z, int m1)

	.align	4
	GLOB(kernel_sgemv_n_4_vs_lib4)
	FUN_START(kernel_sgemv_n_4_vs_lib4)
	

	PROLOGUE


	ZERO_ACC_N


	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_N_4_LIB4
#else
	CALL(inner_kernel_gemv_n_4_lib4)
#endif


	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x4 // beta
	mov		x10, x5 // y

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4_LIB4
#else
	CALL(inner_scale_ab_4_lib4)
#endif


	// store n
	mov		x8, x6 // z
	mov		w9, w7 // m1

#if MACRO_LEVEL>=1
	INNER_STORE_4_VS_LIB4
#else
	CALL(inner_store_4_vs_lib4)
#endif


	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemv_n_4_vs_lib4)





//                            w0        x1             w2         x3         w4      x5         x6            x7         sp+0
// void kernel_sgemv_t_4_lib4(int kmax, double *alpha, int offA, double *A, int sda, double *x, double *beta, double *y, double *z)

	.align	4
	GLOB(kernel_sgemv_t_4_lib4)
	FUN_START(kernel_sgemv_t_4_lib4)
	

	PROLOGUE


	ZERO_ACC_T


	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x3 // A
	mov		w10, w4 // sda
	lsl		w10, w10, #4 // 16*sda
	mov		x11, x5 // x
	mov		w12, w2 // offA

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMV_T_4_LIB4
#else
	CALL(inner_edge_gemv_t_4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_T_4_LIB4
#else
	CALL(inner_kernel_gemv_t_4_lib4)
#endif


	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // y

#if MACRO_LEVEL>=1
	INNER_BLEND_T_4_LIB4
	INNER_SCALE_AB_4_LIB4
#else
	CALL(inner_blend_t_4_lib4)
	CALL(inner_scale_ab_4_lib4)
#endif


	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // z

#if MACRO_LEVEL>=1
	INNER_STORE_4_LIB4
#else
	CALL(inner_store_4_lib4)
#endif


	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemv_t_4_lib4)





//                            w0        x1             w2         x3         w4      x5         x6            x7         sp+0          sp+8
// void kernel_sgemv_t_4_vs_lib4(int kmax, double *alpha, int offA, double *A, int sda, double *x, double *beta, double *y, double *z, int m1)

	.align	4
	GLOB(kernel_sgemv_t_4_vs_lib4)
	FUN_START(kernel_sgemv_t_4_vs_lib4)


	PROLOGUE


	ZERO_ACC_T


	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x3 // A
	mov		w10, w4 // sda
	lsl		w10, w10, #4 // 16*sda
	mov		x11, x5 // x
	mov		w12, w2 // offA

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMV_T_4_LIB4
#else
	CALL(inner_edge_gemv_t_4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_T_4_LIB4
#else
	CALL(inner_kernel_gemv_t_4_lib4)
#endif


	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // y

#if MACRO_LEVEL>=1
	INNER_BLEND_T_4_LIB4
	INNER_SCALE_AB_4_LIB4
#else
	CALL(inner_blend_t_4_lib4)
	CALL(inner_scale_ab_4_lib4)
#endif


	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // z
	ldr		w9, [sp, #(STACKSIZE + 8)] // m1

#if MACRO_LEVEL>=1
	INNER_STORE_4_VS_LIB4
#else
	CALL(inner_store_4_vs_lib4)
#endif


	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemv_t_4_vs_lib4)





//                                 w0     x1         x2                  x3         x4         x5
// void kernel_strsv_ln_inv_4_lib4(int k, double *A, double *inv_diag_A, double *x, double *y, double *z);

	.p2align 4
	GLOB(kernel_strsv_ln_inv_4_lib4)
	FUN_START(kernel_strsv_ln_inv_4_lib4)


	PROLOGUE


	ZERO_ACC_N


	// call inner dgemv kernel n
	mov		w8, w0 // k
	mov		x9, x1  // A
	mov		x10, x3  // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_N_4_LIB4
#else
	CALL(inner_kernel_gemv_n_4_lib4)
#endif


	// call inner blender n
	mov		x8, x4   // y

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4_LIB4
#else
	CALL(inner_scale_m11_4_lib4)
#endif


	// solution
	mov		x8, x1 // A+k*sizeof(double)
	mov		w9, w0 // k
	lsl		w9, w9, #5 // k*sizeof(double)
	add		x8, x8, x9
	mov		x9, x2 // inv_diag_A

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSV_LN_INV_4_LIB4
#else
	CALL(inner_edge_trsv_ln_inv_4_lib4)
#endif


	// store
	mov		x8, x5 // z 

#if MACRO_LEVEL>=1
	INNER_STORE_4_LIB4
#else
	CALL(inner_store_4_lib4)
#endif


	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_strsv_ln_inv_4_lib4)






//                                    w0     x1         x2                  x3         x4         x5         w6      w7
// void kernel_strsv_ln_inv_4_vs_lib4(int k, double *A, double *inv_diag_A, double *x, double *y, double *z, int m1, int n1);

	.p2align 4
	GLOB(kernel_strsv_ln_inv_4_vs_lib4)
	FUN_START(kernel_strsv_ln_inv_4_vs_lib4)
	

	PROLOGUE


	ZERO_ACC_N


	// call inner dgemv kernel n
	mov		w8, w0 // k
	mov		x9, x1  // A
	mov		x10, x3  // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_N_4_LIB4
#else
	CALL(inner_kernel_gemv_n_4_lib4)
#endif


	// call inner blender n
	mov		x8, x4   // y

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4_LIB4
#else
	CALL(inner_scale_m11_4_lib4)
#endif


	// solution
	mov		x8, x1 // A+k*sizeof(double)
	mov		w9, w0 // k
	lsl		w9, w9, #5 // k*sizeof(double)
	add		x8, x8, x9
	mov		x9, x2 // inv_diag_A
	mov		w10, w7 // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSV_LN_INV_4_VS_LIB4
#else
	CALL(inner_edge_trsv_ln_inv_4_vs_lib4)
#endif


	// store
	mov		x8, x5 // z 
	mov		w9, w6 // m1

#if MACRO_LEVEL>=1
	INNER_STORE_4_VS_LIB4
#else
	CALL(inner_store_4_vs_lib4)
#endif


	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_strsv_ln_inv_4_vs_lib4)





#if 0
// TODO
//                                 w0     x1         w2       x3                  x4         x5         x6
// void kernel_strsv_lt_inv_4_lib4(int k, double *A, int sda, double *inv_diag_A, double *x, double *y, double *z);

	.p2align 4
	GLOB(kernel_strsv_lt_inv_4_lib4)
	FUN_START(kernel_strsv_lt_inv_4_lib4)
	

	PROLOGUE


	ZERO_ACC_T


	// call inner dgemv kernel n
	mov		w8, w0 // k
	sub		w8, w8, #4
	mov		x9, x1  // A
	mov		w10, w2 // sda
	lsl		w10, w10, #4 // 16*sda
	add		x9, x9, x10 // A+bs*sda
	mov		x11, x4  // x
	add		x11, x11, #16 // x+4

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_T_4_LIB4
#else
	CALL(inner_kernel_gemv_t_4_lib4)
#endif


	// call inner blender n
	mov		x8, x5   // y

#if MACRO_LEVEL>=1
	INNER_BLEND_T_4_LIB4
	INNER_SCALE_M11_4_LIB4
#else
	INNER_BLEND_T_4_LIB4
	CALL(inner_scale_m11_4_lib4)
#endif


	// solution
	mov		x8, x1 // A
	mov		x9, x3 // inv_diag_A

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSV_LT_INV_4_LIB4
#else
	CALL(inner_edge_trsv_lt_inv_4_lib4)
#endif


	// store
	mov		x8, x6 // z 

#if MACRO_LEVEL>=1
	INNER_STORE_4_LIB4
#else
	CALL(inner_store_4_lib4)
#endif


	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_strsv_lt_inv_4_lib4)
#endif








