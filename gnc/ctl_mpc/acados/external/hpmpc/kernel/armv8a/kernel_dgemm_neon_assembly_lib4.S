/**************************************************************************************************
*                                                                                                 *
* This file is part of HPMPC.                                                                     *
*                                                                                                 *
* HPMPC -- Library for High-Performance implementation of solvers for MPC.                        *
* Copyright (C) 2014-2015 by Technical University of Denmark. All rights reserved.                *
*                                                                                                 *
* HPMPC is free software; you can redistribute it and/or                                          *
* modify it under the terms of the GNU Lesser General Public                                      *
* License as published by the Free Software Foundation; either                                    *
* version 2.1 of the License, or (at your option) any later version.                              *
*                                                                                                 *
* HPMPC is distributed in the hope that it will be useful,                                        *
* but WITHOUT ANY WARRANTY; without even the implied warranty of                                  *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                                            *
* See the GNU Lesser General Public License for more details.                                     *
*                                                                                                 *
* You should have received a copy of the GNU Lesser General Public                                *
* License along with HPMPC; if not, write to the Free Software                                    *
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA                  *
*                                                                                                 *
* Author: Gianluca Frison, giaf (at) dtu.dk                                                       *
*                                                                                                 *
**************************************************************************************************/

// must save (and restore) x18-x29 and v08-v17


	.text

//                                 w0        x1         x2         x3         x4         w5       w6      w7
// void kernel_dgemm_nt_4x4_lib4_S(int kmax, double *A, double *B, double *C, double *D, int alg, int tc, int td)

	.align	4
	.global	kernel_dgemm_nt_4x4_lib4
	.type	kernel_dgemm_nt_4x4_lib4, %function

kernel_dgemm_nt_4x4_lib4:
	
// prologue 

	.align 5
//	add sp, sp, #-(11 * 16)
	add sp, sp, #-(5 * 16)
	stp d8, d9, [sp, #(0 * 16)]
	stp d10, d11, [sp, #(1 * 16)]
	stp d12, d13, [sp, #(2 * 16)]
	stp d14, d15, [sp, #(3 * 16)]
	stp d16, d17, [sp, #(4 * 16)]
//	stp x18, x19, [sp, #(5 * 16)]
//	stp x20, x21, [sp, #(6 * 16)]
//	stp x22, x23, [sp, #(7 * 16)]
//	stp x24, x25, [sp, #(8 * 16)]
//	stp x26, x27, [sp, #(9 * 16)]
//	stp x28, x29, [sp, #(10 * 16)]


	cmp	w0, #0


// zero accumulation registers
	fmov	d16, xzr
	fmov    d17, d16
	fmov    d18, d16
	fmov    d19, d16
	fmov    d20, d16
	fmov    d21, d16
	fmov    d22, d16
	fmov    d23, d16
	fmov    d24, d16
	fmov    d25, d16
	fmov    d26, d16
	fmov    d27, d16
	fmov    d28, d16
	fmov    d29, d16
	fmov    d30, d16
	fmov    d31, d16

// A_0
// d0,  d1,  d2,  d3
//
// B_0
// d4,  d5,  d6,  d7
//
// A_1
// d8,  d9,  d10, d11
//
// B_1
// d12, d13, d14, d15
//
// C
// d16, d20, d24, d28
// d17, d21, d25, d29
// d18, d22, d26, d30
// d19, d23, d27, d31


	ble	.L00_end_matter


	cmp	w0, #3


// prefetch
	ldp   d0, d1, [x1, #(8 * 0)]
	ldp   d2, d3, [x1, #(8 * 2)]
	
	ldp   d4, d5, [x2, #(8 * 0)]
	ldp   d6, d7, [x2, #(8 * 2)]


	ble	.L00_check_clean_loop


.L00_main_loop:

	// 0
	fmadd d16, d0, d4, d16
	fmadd d17, d1, d4, d17
	ldp   d8, d9, [x1, #(8 * 4)]
	fmadd d18, d2, d4, d18
	fmadd d19, d3, d4, d19
	ldp   d10, d11, [x1, #(8 * 6)]

	fmadd d20, d0, d5, d20
	fmadd d21, d1, d5, d21
	ldp   d12, d13, [x2, #(8 * 4)]
	fmadd d22, d2, d5, d22
	fmadd d23, d3, d5, d23
	ldp   d14, d15, [x2, #(8 * 6)]

	fmadd d24, d0, d6, d24
	fmadd d25, d1, d6, d25
	fmadd d26, d2, d6, d26
	fmadd d27, d3, d6, d27

	fmadd d28, d0, d7, d28
	fmadd d29, d1, d7, d29
	ldp   d0, d1, [x1, #(8 * 8)]
	fmadd d30, d2, d7, d30
	fmadd d31, d3, d7, d31
	ldp   d2, d3, [x1, #(8 * 10)]


	// 1
	fmadd d16, d8, d12, d16
	fmadd d17, d9, d12, d17
	ldp   d4, d5, [x2, #(8 * 8)]
	fmadd d18, d10, d12, d18
	fmadd d19, d11, d12, d19
	ldp   d6, d7, [x2, #(8 * 10)]
	
	fmadd d20, d8, d13, d20
	fmadd d21, d9, d13, d21
	sub w0, w0, #4
	fmadd d22, d10, d13, d22
	fmadd d23, d11, d13, d23

	fmadd d24, d8, d14, d24
	fmadd d25, d9, d14, d25
	fmadd d26, d10, d14, d26
	fmadd d27, d11, d14, d27

	fmadd d28, d8, d15, d28
	fmadd d29, d9, d15, d29
	ldp   d8, d9, [x1, #(8 * 12)]
	fmadd d30, d10, d15, d30
	fmadd d31, d11, d15, d31
	ldp   d10, d11, [x1, #(8 * 14)]


	// 2
	fmadd d16, d0, d4, d16
	fmadd d17, d1, d4, d17
	ldp   d12, d13, [x2, #(8 * 12)]
	fmadd d18, d2, d4, d18
	fmadd d19, d3, d4, d19
	ldp   d14, d15, [x2, #(8 * 14)]
	
	fmadd d20, d0, d5, d20
	fmadd d21, d1, d5, d21
	fmadd d22, d2, d5, d22
	fmadd d23, d3, d5, d23

	fmadd d24, d0, d6, d24
	fmadd d25, d1, d6, d25
	fmadd d26, d2, d6, d26
	fmadd d27, d3, d6, d27
	cmp	w0, #3

	fmadd d28, d0, d7, d28
	fmadd d29, d1, d7, d29
	ldp   d0, d1, [x1, #(8 * 16)]
	fmadd d30, d2, d7, d30
	fmadd d31, d3, d7, d31
	ldp   d2, d3, [x1, #(8 * 18)]


	// 3
	fmadd d16, d8, d12, d16
	fmadd d17, d9, d12, d17
	ldp   d4, d5, [x2, #(8 * 16)]
	fmadd d18, d10, d12, d18
	fmadd d19, d11, d12, d19
	ldp   d6, d7, [x2, #(8 * 18)]
	
	fmadd d20, d8, d13, d20
	fmadd d21, d9, d13, d21
	add x1, x1, #(8 * 16)
	fmadd d22, d10, d13, d22
	fmadd d23, d11, d13, d23
	add x2, x2, #(8 * 16)

	fmadd d24, d8, d14, d24
	fmadd d25, d9, d14, d25
	fmadd d26, d10, d14, d26
	fmadd d27, d11, d14, d27
	
	fmadd d28, d8, d15, d28
	fmadd d29, d9, d15, d29
	fmadd d30, d10, d15, d30
	fmadd d31, d11, d15, d31


	bgt	.L00_main_loop



.L00_check_clean_loop:

	cmp	w0, #0

	ble	.L00_end_matter


.L00_clean_loop:

	ldp   d0, d1, [x1, #(8 * 0)]
	ldp   d2, d3, [x1, #(8 * 2)]
	
	ldp   d4, d5, [x2, #(8 * 0)]
	ldp   d6, d7, [x2, #(8 * 2)]

	fmadd d16, d0, d4, d16
	fmadd d17, d1, d4, d17
	add x1, x1, #(8 * 4)
	fmadd d18, d2, d4, d18
	fmadd d19, d3, d4, d19
	add x2, x2, #(8 * 4)

	fmadd d20, d0, d5, d20
	fmadd d21, d1, d5, d21
	sub w0, w0, #1
	fmadd d22, d2, d5, d22
	fmadd d23, d3, d5, d23
	cmp	w0, #0

	fmadd d24, d0, d6, d24
	fmadd d25, d1, d6, d25
	fmadd d26, d2, d6, d26
	fmadd d27, d3, d6, d27

	fmadd d28, d0, d7, d28
	fmadd d29, d1, d7, d29
	fmadd d30, d2, d7, d30
	fmadd d31, d3, d7, d31

	bgt	.L00_clean_loop



.L00_end_matter:

	cmp w5, #0

	beq .L00_store

	cmp w6, #0

	bne .L00_load_t

// load_n C
	ldp d0, d1, [x3, #(0 * 16)]
	ldp d2, d3, [x3, #(1 * 16)]
	ldp d4, d5, [x3, #(2 * 16)]
	ldp d6, d7, [x3, #(3 * 16)]
	ldp d8, d9, [x3, #(4 * 16)]
	ldp d10, d11, [x3, #(5 * 16)]
	ldp d12, d13, [x3, #(6 * 16)]
	ldp d14, d15, [x3, #(7 * 16)]

	b .L00_update

.L00_load_t:

// load_t C
	ldp d0, d4, [x3, #(0 * 16)]
	ldp d8, d12, [x3, #(1 * 16)]
	ldp d1, d5, [x3, #(2 * 16)]
	ldp d9, d13, [x3, #(3 * 16)]
	ldp d2, d6, [x3, #(4 * 16)]
	ldp d10, d14, [x3, #(5 * 16)]
	ldp d3, d7, [x3, #(6 * 16)]
	ldp d11, d15, [x3, #(7 * 16)]

.L00_update:

	cmp w5, #0

	blt .L00_sub

	fadd d16, d0, d16
	fadd d17, d1, d17
	fadd d18, d2, d18
	fadd d19, d3, d19
	fadd d20, d4, d20
	fadd d21, d5, d21
	fadd d22, d6, d22
	fadd d23, d7, d23
	fadd d24, d8, d24
	fadd d25, d9, d25
	fadd d26, d10, d26
	fadd d27, d11, d27
	fadd d28, d12, d28
	fadd d29, d13, d29
	fadd d30, d14, d30
	fadd d31, d15, d31

	b .L00_store

.L00_sub:

	fsub d16, d0, d16
	fsub d17, d1, d17
	fsub d18, d2, d18
	fsub d19, d3, d19
	fsub d20, d4, d20
	fsub d21, d5, d21
	fsub d22, d6, d22
	fsub d23, d7, d23
	fsub d24, d8, d24
	fsub d25, d9, d25
	fsub d26, d10, d26
	fsub d27, d11, d27
	fsub d28, d12, d28
	fsub d29, d13, d29
	fsub d30, d14, d30
	fsub d31, d15, d31

.L00_store:

	cmp w7, #0

	bne .L00_store_t
	
// store_n D
	stp d16, d17, [x4, #(0 * 16)]
	stp d18, d19, [x4, #(1 * 16)]
	stp d20, d21, [x4, #(2 * 16)]
	stp d22, d23, [x4, #(3 * 16)]
	stp d24, d25, [x4, #(4 * 16)]
	stp d26, d27, [x4, #(5 * 16)]
	stp d28, d29, [x4, #(6 * 16)]
	stp d30, d31, [x4, #(7 * 16)]

	b .L00_epilogue

.L00_store_t:

// store_t D
	stp d16, d20, [x4, #(0 * 16)]
	stp d24, d28, [x4, #(1 * 16)]
	stp d17, d21, [x4, #(2 * 16)]
	stp d25, d29, [x4, #(3 * 16)]
	stp d18, d22, [x4, #(4 * 16)]
	stp d26, d30, [x4, #(5 * 16)]
	stp d19, d23, [x4, #(6 * 16)]
	stp d27, d31, [x4, #(7 * 16)]


.L00_epilogue:
// epilogue

	ldp d8, d9, [sp, #(0 * 16)]
	ldp d10, d11, [sp, #(1 * 16)]
	ldp d12, d13, [sp, #(2 * 16)]
	ldp d14, d15, [sp, #(3 * 16)]
	ldp d16, d17, [sp, #(4 * 16)]
//	ldp x18, x19, [sp, #(5 * 16)]
//	ldp x20, x21, [sp, #(6 * 16)]
//	ldp x22, x23, [sp, #(7 * 16)]
//	ldp x24, x25, [sp, #(8 * 16)]
//	ldp x26, x27, [sp, #(9 * 16)]
//	ldp x28, x29, [sp, #(10 * 16)]
//	add sp, sp, #(11 * 16)
	add sp, sp, #(5 * 16)

	ret







//                               w0        x1         w2       x3         x4         w5       x6         w7       sp+0     sp+8    sp+16
// void kernel_dgemm_nt_8x4_lib4(int kmax, double *A, int sda, double *B, double *C, int sdc, double *D, int sdd, int alg, int tc, int td)

	.align	4
	.global	kernel_dgemm_nt_8x4_lib4
	.type	kernel_dgemm_nt_8x4_lib4, %function

kernel_dgemm_nt_8x4_lib4:

	prfm  PLDL1KEEP, [x3, #0]
	prfm  PLDL1KEEP, [x1, #0]

// load arguments from stack
	ldr w8, [sp, #0]
	ldr w9, [sp, #8]
	ldr w10, [sp, #16]

	prfm  PLDL1KEEP, [x3, #64]
	prfm  PLDL1KEEP, [x1, #64]

	lsl w2, w2, #5 // sda * bs * 8
	lsl w5, w5, #5 // sdc * bs * 8
	lsl w7, w7, #5 // sdd * bs * 8

	add x11, x1, x2 // A1
	add x12, x4, x5 // C1
	add x13, x6, x7 // D1

	prfm  PLDL1KEEP, [x11, #0]
	prfm  PLDL1KEEP, [x11, #64]
	
// prologue 

	.align 5
//	add sp, sp, #-(11 * 16)
	add sp, sp, #-(5 * 16)
	stp d8, d9, [sp, #(0 * 16)]
	stp d10, d11, [sp, #(1 * 16)]
	stp d12, d13, [sp, #(2 * 16)]
	stp d14, d15, [sp, #(3 * 16)]
	stp d16, d17, [sp, #(4 * 16)]
//	stp x18, x19, [sp, #(5 * 16)]
//	stp x20, x21, [sp, #(6 * 16)]
//	stp x22, x23, [sp, #(7 * 16)]
//	stp x24, x25, [sp, #(8 * 16)]
//	stp x26, x27, [sp, #(9 * 16)]
//	stp x28, x29, [sp, #(10 * 16)]



	cmp	w0, #0


// zero accumulation registers
	fmov	d16, xzr
	fmov    d17, d16
	fmov    d18, d16
	fmov    d19, d16
	fmov    d20, d16
	fmov    d21, d16
	fmov    d22, d16
	fmov    d23, d16
	fmov    d24, d16
	fmov    d25, d16
	fmov    d26, d16
	fmov    d27, d16
	fmov    d28, d16
	fmov    d29, d16
	fmov    d30, d16
	fmov    d31, d16

// A0_0
// v0.2d[0],  v0.2d[1],  v1.2d[0],  v1.2d[1]
//
// A1_0
// v2.2d[0],  v2.2d[1],  v3.2d[0],  v3.2d[1]
//
// B_0
// v4.2d[0],  v4.2d[1],  v5.2d[0],  v5.2d[1]
//
// A0_1
// v6.2d[0],  v6.2d[1],  v7.2d[0],  v7.2d[1]
//
// A1_1
// v8.2d[0],  v8.2d[1],  v9.2d[0],  v9.2d[1]
//
// B_1
// v10.2d[0], v10.2d[1], v11.2d[0], v11.2d[1]
//
// C0
// v16.2d[0], v18.2d[0], v20.2d[0], v22.2d[0]
// v16.2d[1], v18.2d[1], v20.2d[1], v22.2d[1]
// v17.2d[0], v19.2d[0], v21.2d[0], v23.2d[0]
// v17.2d[1], v19.2d[1], v21.2d[1], v23.2d[1]
//
// C0_t
// v16.2d[0], v20.2d[0], v17.2d[0], v21.2d[0]
// v16.2d[1], v20.2d[1], v17.2d[1], v21.2d[1]
// v18.2d[0], v22.2d[0], v19.2d[0], v23.2d[0]
// v18.2d[1], v22.2d[1], v19.2d[1], v23.2d[1]
//
// C1
// v24.2d[0], v26.2d[0], v28.2d[0], v30.2d[0]
// v24.2d[1], v26.2d[1], v28.2d[1], v30.2d[1]
// v25.2d[0], v27.2d[0], v29.2d[0], v31.2d[0]
// v25.2d[1], v27.2d[1], v29.2d[1], v31.2d[1]


	ble	.L01_end_matter


	cmp	w0, #3


// prefetch
ld1   {v0.2d, v1.2d}, [x1], #32
ld1   {v4.2d, v5.2d}, [x3], #32
ld1   {v2.2d, v3.2d}, [x11], #32

	ble	.L01_check_clean_loop


.L01_main_loop:

	// 0
	ld1   {v6.2d, v7.2d}, [x1], #32
	fmla  v16.2d, v0.2d, v4.2d[0]
	ld1   {v10.2d, v11.2d}, [x3], #32
	fmla  v17.2d, v1.2d, v4.2d[0]
	ld1   {v8.2d, v9.2d}, [x11], #32
	fmla  v24.2d, v2.2d, v4.2d[0]
	prfm  PLDL1KEEP, [x3, #64]
	fmla  v25.2d, v3.2d, v4.2d[0]

	prfm  PLDL1KEEP, [x1, #64]
	fmla  v18.2d, v0.2d, v4.2d[1]
	prfm  PLDL1KEEP, [x11, #64]
	fmla  v19.2d, v1.2d, v4.2d[1]
	fmla  v26.2d, v2.2d, v4.2d[1]
	fmla  v27.2d, v3.2d, v4.2d[1]

	fmla  v20.2d, v0.2d, v5.2d[0]
	fmla  v21.2d, v1.2d, v5.2d[0]
	sub w0, w0, #4
	fmla  v28.2d, v2.2d, v5.2d[0]
	fmla  v29.2d, v3.2d, v5.2d[0]

	fmla  v22.2d, v0.2d, v5.2d[1]
	fmla  v23.2d, v1.2d, v5.2d[1]
	ld1   {v0.2d, v1.2d}, [x1], #32
	fmla  v30.2d, v2.2d, v5.2d[1]
	fmla  v31.2d, v3.2d, v5.2d[1]
	ld1   {v4.2d, v5.2d}, [x3], #32


	// 1
	fmla  v16.2d, v6.2d, v10.2d[0]
	ld1   {v2.2d, v3.2d}, [x11], #32
	fmla  v17.2d, v7.2d, v10.2d[0]
	fmla  v24.2d, v8.2d, v10.2d[0]
	fmla  v25.2d, v9.2d, v10.2d[0]

	fmla  v18.2d, v6.2d, v10.2d[1]
	fmla  v19.2d, v7.2d, v10.2d[1]
	fmla  v26.2d, v8.2d, v10.2d[1]
	fmla  v27.2d, v9.2d, v10.2d[1]

	fmla  v20.2d, v6.2d, v11.2d[0]
	fmla  v21.2d, v7.2d, v11.2d[0]
	fmla  v28.2d, v8.2d, v11.2d[0]
	fmla  v29.2d, v9.2d, v11.2d[0]

	fmla  v22.2d, v6.2d, v11.2d[1]
	fmla  v23.2d, v7.2d, v11.2d[1]
	ld1   {v6.2d, v7.2d}, [x1], #32
	fmla  v30.2d, v8.2d, v11.2d[1]
	fmla  v31.2d, v9.2d, v11.2d[1]
	ld1   {v10.2d, v11.2d}, [x3], #32


	// 2
	fmla  v16.2d, v0.2d, v4.2d[0]
	ld1   {v8.2d, v9.2d}, [x11], #32
	fmla  v17.2d, v1.2d, v4.2d[0]
	prfm  PLDL1KEEP, [x3, #64]
	fmla  v24.2d, v2.2d, v4.2d[0]
	prfm  PLDL1KEEP, [x1, #64]
	fmla  v25.2d, v3.2d, v4.2d[0]

	prfm  PLDL1KEEP, [x11, #64]
	fmla  v18.2d, v0.2d, v4.2d[1]
	fmla  v19.2d, v1.2d, v4.2d[1]
	fmla  v26.2d, v2.2d, v4.2d[1]
	fmla  v27.2d, v3.2d, v4.2d[1]

	fmla  v20.2d, v0.2d, v5.2d[0]
	fmla  v21.2d, v1.2d, v5.2d[0]
	cmp	w0, #3
	fmla  v28.2d, v2.2d, v5.2d[0]
	fmla  v29.2d, v3.2d, v5.2d[0]

	fmla  v22.2d, v0.2d, v5.2d[1]
	fmla  v23.2d, v1.2d, v5.2d[1]
	ld1   {v0.2d, v1.2d}, [x1], #32
	fmla  v30.2d, v2.2d, v5.2d[1]
	fmla  v31.2d, v3.2d, v5.2d[1]
	ld1   {v4.2d, v5.2d}, [x3], #32


	// 3
	fmla  v16.2d, v6.2d, v10.2d[0]
	ld1   {v2.2d, v3.2d}, [x11], #32
	fmla  v17.2d, v7.2d, v10.2d[0]
	fmla  v24.2d, v8.2d, v10.2d[0]
	fmla  v25.2d, v9.2d, v10.2d[0]

	fmla  v18.2d, v6.2d, v10.2d[1]
	fmla  v19.2d, v7.2d, v10.2d[1]
	fmla  v26.2d, v8.2d, v10.2d[1]
	fmla  v27.2d, v9.2d, v10.2d[1]

	fmla  v20.2d, v6.2d, v11.2d[0]
	fmla  v21.2d, v7.2d, v11.2d[0]
	fmla  v28.2d, v8.2d, v11.2d[0]
	fmla  v29.2d, v9.2d, v11.2d[0]

	fmla  v22.2d, v6.2d, v11.2d[1]
	fmla  v23.2d, v7.2d, v11.2d[1]
	fmla  v30.2d, v8.2d, v11.2d[1]
	fmla  v31.2d, v9.2d, v11.2d[1]


	bgt	.L01_main_loop



.L01_check_clean_loop:

	cmp	w0, #0

	ble	.L01_end_matter

	b .L01_clean_loop_no_load

.L01_clean_loop:

	ld1   {v0.2d, v1.2d}, [x1], #32
	ld1   {v4.2d, v5.2d}, [x3], #32
	ld1   {v2.2d, v3.2d}, [x11], #32

.L01_clean_loop_no_load:

	// 0
	fmla  v16.2d, v0.2d, v4.2d[0]
	fmla  v17.2d, v1.2d, v4.2d[0]
	fmla  v24.2d, v2.2d, v4.2d[0]
	fmla  v25.2d, v3.2d, v4.2d[0]

	sub w0, w0, #1

	fmla  v18.2d, v0.2d, v4.2d[1]
	fmla  v19.2d, v1.2d, v4.2d[1]
	fmla  v26.2d, v2.2d, v4.2d[1]
	fmla  v27.2d, v3.2d, v4.2d[1]

	cmp	w0, #0

	fmla  v20.2d, v0.2d, v5.2d[0]
	fmla  v21.2d, v1.2d, v5.2d[0]
	fmla  v28.2d, v2.2d, v5.2d[0]
	fmla  v29.2d, v3.2d, v5.2d[0]

	fmla  v22.2d, v0.2d, v5.2d[1]
	fmla  v23.2d, v1.2d, v5.2d[1]
	fmla  v30.2d, v2.2d, v5.2d[1]
	fmla  v31.2d, v3.2d, v5.2d[1]

	bgt	.L01_clean_loop



.L01_end_matter:

	cmp w8, #0

	beq .L01_store

	cmp w9, #0

	bne .L01_load_t

// load_n C
	ld1 {v0.2d, v1.2d, v2.2d, v3.2d}, [x4], #64
	ld1 {v4.2d, v5.2d, v6.2d, v7.2d}, [x4], #64

	ld1 {v8.2d, v9.2d, v10.2d, v11.2d}, [x12], #64
	ld1 {v12.2d, v13.2d, v14.2d, v15.2d}, [x12], #64

	b .L01_update

.L01_load_t:

// load_t C
	ld4 {v0.2d, v1.2d, v2.2d, v3.2d}, [x4], #64
	ld4 {v4.2d, v5.2d, v6.2d, v7.2d}, [x4], #64

//	sub sp, sp, #32
//	st1 {v4.2d, v5.2d}, [sp]
	mov v8.16b, v4.16b
	mov v9.16b, v5.16b

	mov v4.16b, v2.16b
	mov v5.16b, v6.16b

	mov v2.16b, v1.16b
	mov v6.16b, v3.16b

//	ld1 {v1.2d}, [sp], #16
//	ld1 {v3.2d}, [sp], #16
	mov v1.16b, v8.16b
	mov v3.16b, v9.16b

	ld4 {v8.2d, v9.2d, v10.2d, v11.2d}, [x4], #64
	ld4 {v12.2d, v13.2d, v14.2d, v15.2d}, [x4], #64

	sub sp, sp, #32
	st1 {v12.2d, v13.2d}, [sp]

	mov v12.16b, v10.16b
	mov v13.16b, v14.16b

	mov v10.16b, v9.16b
	mov v14.16b, v11.16b

	ld1 {v9.2d}, [sp], #16
	ld1 {v11.2d}, [sp], #16

.L01_update:

	cmp w8, #0

	blt .L01_sub

	fadd v16.2d, v0.2d, v16.2d
	fadd v17.2d, v1.2d, v17.2d
	fadd v18.2d, v2.2d, v18.2d
	fadd v19.2d, v3.2d, v19.2d
	fadd v20.2d, v4.2d, v20.2d
	fadd v21.2d, v5.2d, v21.2d
	fadd v22.2d, v6.2d, v22.2d
	fadd v23.2d, v7.2d, v23.2d

	fadd v24.2d, v8.2d, v24.2d
	fadd v25.2d, v9.2d, v25.2d
	fadd v26.2d, v10.2d, v26.2d
	fadd v27.2d, v11.2d, v27.2d
	fadd v28.2d, v12.2d, v28.2d
	fadd v29.2d, v13.2d, v29.2d
	fadd v30.2d, v14.2d, v30.2d
	fadd v31.2d, v15.2d, v31.2d

	b .L01_store

.L01_sub:

	fsub v16.2d, v0.2d, v16.2d
	fsub v17.2d, v1.2d, v17.2d
	fsub v18.2d, v2.2d, v18.2d
	fsub v19.2d, v3.2d, v19.2d
	fsub v20.2d, v4.2d, v20.2d
	fsub v21.2d, v5.2d, v21.2d
	fsub v22.2d, v6.2d, v22.2d
	fsub v23.2d, v7.2d, v23.2d

	fsub v24.2d, v8.2d, v24.2d
	fsub v25.2d, v9.2d, v25.2d
	fsub v26.2d, v10.2d, v26.2d
	fsub v27.2d, v11.2d, v27.2d
	fsub v28.2d, v12.2d, v28.2d
	fsub v29.2d, v13.2d, v29.2d
	fsub v30.2d, v14.2d, v30.2d
	fsub v31.2d, v15.2d, v31.2d

.L01_store:

	cmp w10, #0

	bne .L01_store_t
	
// store_n D
	st1 {v16.2d, v17.2d, v18.2d, v19.2d}, [x6], #64
	st1 {v20.2d, v21.2d, v22.2d, v23.2d}, [x6], #64

	st1 {v24.2d, v25.2d, v26.2d, v27.2d}, [x13], #64
	st1 {v28.2d, v29.2d, v30.2d, v31.2d}, [x13], #64

	b .L01_epilogue

.L01_store_t:
	
// store_t D
	mov v0.16b, v17.16b
	mov v1.16b, v19.16b
	mov v17.16b, v18.16b
	mov v19.16b, v22.16b
	mov v18.16b, v20.16b
	mov v22.16b, v21.16b
	mov v20.16b, v0.16b
	mov v21.16b, v1.16b

	st4 {v16.2d, v17.2d, v18.2d, v19.2d}, [x6], #64
	st4 {v20.2d, v21.2d, v22.2d, v23.2d}, [x6], #64

	mov v0.16b, v25.16b
	mov v1.16b, v27.16b
	mov v25.16b, v26.16b
	mov v27.16b, v30.16b
	mov v26.16b, v28.16b
	mov v30.16b, v29.16b
	mov v28.16b, v0.16b
	mov v29.16b, v1.16b

	st4 {v24.2d, v25.2d, v26.2d, v27.2d}, [x6], #64
	st4 {v28.2d, v29.2d, v30.2d, v31.2d}, [x6], #64


.L01_epilogue:
// epilogue

	ldp d8, d9, [sp, #(0 * 16)]
	ldp d10, d11, [sp, #(1 * 16)]
	ldp d12, d13, [sp, #(2 * 16)]
	ldp d14, d15, [sp, #(3 * 16)]
	ldp d16, d17, [sp, #(4 * 16)]
//	ldp x18, x19, [sp, #(5 * 16)]
//	ldp x20, x21, [sp, #(6 * 16)]
//	ldp x22, x23, [sp, #(7 * 16)]
//	ldp x24, x25, [sp, #(8 * 16)]
//	ldp x26, x27, [sp, #(9 * 16)]
//	ldp x28, x29, [sp, #(10 * 16)]
//	add sp, sp, #(11 * 16)
	add sp, sp, #(5 * 16)

	mov w0, w2

	ret



