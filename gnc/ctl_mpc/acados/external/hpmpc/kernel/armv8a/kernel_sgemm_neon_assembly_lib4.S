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

//                                   w0        x1        x2        x3        x4        w5       w6      w7
// void kernel_sgemm_nt_4x4_lib4_new(int kmax, float *A, float *B, float *C, float *D, int alg, int tc, int td)

	.align	4
	.global	kernel_sgemm_nt_4x4_lib4_new
	.type	kernel_sgemm_nt_4x4_lib4_new, %function

kernel_sgemm_nt_4x4_lib4_new:
	
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
	ldp   s0, s1, [x1, #(4 * 0)]
	ldp   s2, s3, [x1, #(4 * 2)]
	
	ldp   s4, s5, [x2, #(4 * 0)]
	ldp   s6, s7, [x2, #(4 * 2)]


	ble	.L00_check_clean_loop


.L00_main_loop:

	// 0
	fmadd s16, s0, s4, s16
	fmadd s17, s1, s4, s17
	ldp   s8, s9, [x1, #(4 * 4)]
	fmadd s18, s2, s4, s18
	fmadd s19, s3, s4, s19
	ldp   s10, s11, [x1, #(4 * 6)]

	fmadd s20, s0, s5, s20
	fmadd s21, s1, s5, s21
	ldp   s12, s13, [x2, #(4 * 4)]
	fmadd s22, s2, s5, s22
	fmadd s23, s3, s5, s23
	ldp   s14, s15, [x2, #(4 * 6)]

	fmadd s24, s0, s6, s24
	fmadd s25, s1, s6, s25
	fmadd s26, s2, s6, s26
	fmadd s27, s3, s6, s27

	fmadd s28, s0, s7, s28
	fmadd s29, s1, s7, s29
	ldp   s0, s1, [x1, #(4 * 8)]
	fmadd s30, s2, s7, s30
	fmadd s31, s3, s7, s31
	ldp   s2, s3, [x1, #(4 * 10)]


	// 1
	fmadd s16, s8, s12, s16
	fmadd s17, s9, s12, s17
	ldp   s4, s5, [x2, #(4 * 8)]
	fmadd s18, s10, s12, s18
	fmadd s19, s11, s12, s19
	ldp   s6, s7, [x2, #(4 * 10)]
	
	fmadd s20, s8, s13, s20
	fmadd s21, s9, s13, s21
	sub w0, w0, #4
	fmadd s22, s10, s13, s22
	fmadd s23, s11, s13, s23

	fmadd s24, s8, s14, s24
	fmadd s25, s9, s14, s25
	fmadd s26, s10, s14, s26
	fmadd s27, s11, s14, s27

	fmadd s28, s8, s15, s28
	fmadd s29, s9, s15, s29
	ldp   s8, s9, [x1, #(4 * 12)]
	fmadd s30, s10, s15, s30
	fmadd s31, s11, s15, s31
	ldp   s10, s11, [x1, #(4 * 14)]


	// 2
	fmadd s16, s0, s4, s16
	fmadd s17, s1, s4, s17
	ldp   s12, s13, [x2, #(4 * 12)]
	fmadd s18, s2, s4, s18
	fmadd s19, s3, s4, s19
	ldp   s14, s15, [x2, #(4 * 14)]
	
	fmadd s20, s0, s5, s20
	fmadd s21, s1, s5, s21
	fmadd s22, s2, s5, s22
	fmadd s23, s3, s5, s23

	fmadd s24, s0, s6, s24
	fmadd s25, s1, s6, s25
	fmadd s26, s2, s6, s26
	fmadd s27, s3, s6, s27
	cmp	w0, #3

	fmadd s28, s0, s7, s28
	fmadd s29, s1, s7, s29
	ldp   s0, s1, [x1, #(4 * 16)]
	fmadd s30, s2, s7, s30
	fmadd s31, s3, s7, s31
	ldp   s2, s3, [x1, #(4 * 18)]


	// 3
	fmadd s16, s8, s12, s16
	fmadd s17, s9, s12, s17
	ldp   s4, s5, [x2, #(4 * 16)]
	fmadd s18, s10, s12, s18
	fmadd s19, s11, s12, s19
	ldp   s6, s7, [x2, #(4 * 18)]
	
	fmadd s20, s8, s13, s20
	fmadd s21, s9, s13, s21
	add x1, x1, #(4 * 16)
	fmadd s22, s10, s13, s22
	fmadd s23, s11, s13, s23
	add x2, x2, #(4 * 16)

	fmadd s24, s8, s14, s24
	fmadd s25, s9, s14, s25
	fmadd s26, s10, s14, s26
	fmadd s27, s11, s14, s27
	
	fmadd s28, s8, s15, s28
	fmadd s29, s9, s15, s29
	fmadd s30, s10, s15, s30
	fmadd s31, s11, s15, s31


	bgt	.L00_main_loop



.L00_check_clean_loop:

	cmp	w0, #0

	ble	.L00_end_matter


.L00_clean_loop:

	ldp   s0, s1, [x1, #(4 * 0)]
	ldp   s2, s3, [x1, #(4 * 2)]
	
	ldp   s4, s5, [x2, #(4 * 0)]
	ldp   s6, s7, [x2, #(4 * 2)]

	fmadd s16, s0, s4, s16
	fmadd s17, s1, s4, s17
	add x1, x1, #(4 * 4)
	fmadd s18, s2, s4, s18
	fmadd s19, s3, s4, s19
	add x2, x2, #(4 * 4)

	fmadd s20, s0, s5, s20
	fmadd s21, s1, s5, s21
	sub w0, w0, #1
	fmadd s22, s2, s5, s22
	fmadd s23, s3, s5, s23
	cmp	w0, #0

	fmadd s24, s0, s6, s24
	fmadd s25, s1, s6, s25
	fmadd s26, s2, s6, s26
	fmadd s27, s3, s6, s27

	fmadd s28, s0, s7, s28
	fmadd s29, s1, s7, s29
	fmadd s30, s2, s7, s30
	fmadd s31, s3, s7, s31

	bgt	.L00_clean_loop



.L00_end_matter:

	cmp w5, #0

	beq .L00_store

	cmp w6, #0

	bne .L00_load_t

// load_n C
	ldp s0, s1, [x3, #(0 * 8)]
	ldp s2, s3, [x3, #(1 * 8)]
	ldp s4, s5, [x3, #(2 * 8)]
	ldp s6, s7, [x3, #(3 * 8)]
	ldp s8, s9, [x3, #(4 * 8)]
	ldp s10, s11, [x3, #(5 * 8)]
	ldp s12, s13, [x3, #(6 * 8)]
	ldp s14, s15, [x3, #(7 * 8)]

	b .L00_update

.L00_load_t:

// load_t C
	ldp s0, s4, [x3, #(0 * 8)]
	ldp s8, s12, [x3, #(1 * 8)]
	ldp s1, s5, [x3, #(2 * 8)]
	ldp s9, s13, [x3, #(3 * 8)]
	ldp s2, s6, [x3, #(4 * 8)]
	ldp s10, s14, [x3, #(5 * 8)]
	ldp s3, s7, [x3, #(6 * 8)]
	ldp s11, s15, [x3, #(7 * 8)]

.L00_update:

	cmp w5, #0

	blt .L00_sub

	fadd s16, s0, s16
	fadd s17, s1, s17
	fadd s18, s2, s18
	fadd s19, s3, s19
	fadd s20, s4, s20
	fadd s21, s5, s21
	fadd s22, s6, s22
	fadd s23, s7, s23
	fadd s24, s8, s24
	fadd s25, s9, s25
	fadd s26, s10, s26
	fadd s27, s11, s27
	fadd s28, s12, s28
	fadd s29, s13, s29
	fadd s30, s14, s30
	fadd s31, s15, s31

	b .L00_store

.L00_sub:

	fsub s16, s0, s16
	fsub s17, s1, s17
	fsub s18, s2, s18
	fsub s19, s3, s19
	fsub s20, s4, s20
	fsub s21, s5, s21
	fsub s22, s6, s22
	fsub s23, s7, s23
	fsub s24, s8, s24
	fsub s25, s9, s25
	fsub s26, s10, s26
	fsub s27, s11, s27
	fsub s28, s12, s28
	fsub s29, s13, s29
	fsub s30, s14, s30
	fsub s31, s15, s31

.L00_store:

	cmp w7, #0

	bne .L00_store_t
	
// store_n D
	stp s16, s17, [x4, #(0 * 8)]
	stp s18, s19, [x4, #(1 * 8)]
	stp s20, s21, [x4, #(2 * 8)]
	stp s22, s23, [x4, #(3 * 8)]
	stp s24, s25, [x4, #(4 * 8)]
	stp s26, s27, [x4, #(5 * 8)]
	stp s28, s29, [x4, #(6 * 8)]
	stp s30, s31, [x4, #(7 * 8)]

	b .L00_epilogue

.L00_store_t:

// store_t D
	stp s16, s20, [x4, #(0 * 8)]
	stp s24, s28, [x4, #(1 * 8)]
	stp s17, s21, [x4, #(2 * 8)]
	stp s25, s29, [x4, #(3 * 8)]
	stp s18, s22, [x4, #(4 * 8)]
	stp s26, s30, [x4, #(5 * 8)]
	stp s19, s23, [x4, #(6 * 8)]
	stp s27, s31, [x4, #(7 * 8)]


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







//                                   w0        x1        w2       x3        x4        w5       x6        w7       sp+0     sp+8    sp+16
// void kernel_sgemm_nt_8x4_lib4_new(int kmax, float *A, int sda, float *B, float *C, int sdc, float *D, int sdd, int alg, int tc, int td)

	.align	4
	.global	kernel_sgemm_nt_8x4_lib4_new
	.type	kernel_sgemm_nt_8x4_lib4_new, %function

kernel_sgemm_nt_8x4_lib4_new:

//	prfm  PLDL1KEEP, [x3, #0]
//	prfm  PLDL1KEEP, [x1, #0]

// load arguments from stack
	ldr w8, [sp, #0]
	ldr w9, [sp, #8]
	ldr w10, [sp, #16]

//	prfm  PLDL1KEEP, [x3, #64]
//	prfm  PLDL1KEEP, [x1, #64]

	lsl w2, w2, #4 // sda * bs * 4
	lsl w5, w5, #4 // sdc * bs * 4
	lsl w7, w7, #4 // sdd * bs * 4

	add x11, x1, x2 // A1
	add x12, x4, x5 // C1
	add x13, x6, x7 // D1

//	prfm  PLDL1KEEP, [x11, #0]
//	prfm  PLDL1KEEP, [x11, #64]
	
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


// zero accumulation registers (upper 64-bit are zeroed automatically)
	fmov	d16, xzr
	fmov    d17, d16
	fmov    d18, d16
	fmov    d19, d16
	fmov    d20, d16
	fmov    d21, d16
	fmov    d22, d16
	fmov    d23, d16
//	fmov    d24, d16
//	fmov    d25, d16
//	fmov    d26, d16
//	fmov    d27, d16
//	fmov    d28, d16
//	fmov    d29, d16
//	fmov    d30, d16
//	fmov    d31, d16

// A0_0
// v0.4s[0],  v0.4s[1],  v0.4s[2],  v0.4s[3]
//
// A1_0
// v2.4s[0],  v2.4s[1],  v2.4s[2],  v2.4s[3]
//
// B_0
// v4.4s[0],  v4.4s[1],  v4.4s[2],  v4.4s[3]
//
// C0
// v16.4s[0], v17.4s[0], v18.4s[0], v19.4s[0]
// v16.4s[1], v17.4s[1], v18.4s[1], v19.4s[1]
// v16.4s[2], v17.4s[0], v18.4s[0], v19.4s[0]
// v16.4s[3], v17.4s[1], v18.4s[1], v19.4s[1]
//
// C1
// v20.4s[0], v21.4s[0], v22.4s[0], v23.4s[0]
// v20.4s[1], v21.4s[1], v22.4s[1], v23.4s[1]
// v20.4s[2], v21.4s[0], v22.4s[0], v23.4s[0]
// v20.4s[3], v21.4s[1], v22.4s[1], v23.4s[1]


	ble	.L01_end_matter


	cmp	w0, #3

	// prefetch

	ble	.L01_check_clean_loop


.L01_main_loop:

	// 0
	ld1   {v0.4s, v1.4s}, [x1], #32
	ld1   {v4.4s, v5.4s}, [x3], #32
	ld1   {v2.4s, v3.4s}, [x11], #32

	fmla  v16.4s, v0.4s, v4.4s[0]
	fmla  v17.4s, v0.4s, v4.4s[1]
	fmla  v18.4s, v0.4s, v4.4s[2]
	fmla  v19.4s, v0.4s, v4.4s[3]

	fmla  v20.4s, v2.4s, v4.4s[0]
	fmla  v21.4s, v2.4s, v4.4s[1]
	fmla  v22.4s, v2.4s, v4.4s[2]
	fmla  v23.4s, v2.4s, v4.4s[3]

	sub w0, w0, #4

	// 1
	fmla  v16.4s, v1.4s, v5.4s[0]
	fmla  v17.4s, v1.4s, v5.4s[1]
	fmla  v18.4s, v1.4s, v5.4s[2]
	fmla  v19.4s, v1.4s, v5.4s[3]

	fmla  v20.4s, v3.4s, v5.4s[0]
	fmla  v21.4s, v3.4s, v5.4s[1]
	fmla  v22.4s, v3.4s, v5.4s[2]
	fmla  v23.4s, v3.4s, v5.4s[3]


	// 2
	ld1   {v0.4s, v1.4s}, [x1], #32
	ld1   {v4.4s, v5.4s}, [x3], #32
	ld1   {v2.4s, v3.4s}, [x11], #32

	fmla  v16.4s, v0.4s, v4.4s[0]
	fmla  v17.4s, v0.4s, v4.4s[1]
	fmla  v18.4s, v0.4s, v4.4s[2]
	fmla  v19.4s, v0.4s, v4.4s[3]

	fmla  v20.4s, v2.4s, v4.4s[0]
	fmla  v21.4s, v2.4s, v4.4s[1]
	fmla  v22.4s, v2.4s, v4.4s[2]
	fmla  v23.4s, v2.4s, v4.4s[3]

	cmp	w0, #3

	// 3
	fmla  v16.4s, v1.4s, v5.4s[0]
	fmla  v17.4s, v1.4s, v5.4s[1]
	fmla  v18.4s, v1.4s, v5.4s[2]
	fmla  v19.4s, v1.4s, v5.4s[3]

	fmla  v20.4s, v3.4s, v5.4s[0]
	fmla  v21.4s, v3.4s, v5.4s[1]
	fmla  v22.4s, v3.4s, v5.4s[2]
	fmla  v23.4s, v3.4s, v5.4s[3]


	bgt	.L01_main_loop



.L01_check_clean_loop:

	cmp	w0, #0

	ble	.L01_end_matter

	b .L01_clean_loop_no_load

.L01_clean_loop:

	// prefetch

.L01_clean_loop_no_load:

	// 0
	ld1   {v0.4s}, [x1], #16
	ld1   {v4.4s}, [x3], #16
	ld1   {v2.4s}, [x11], #16

	fmla  v16.4s, v0.4s, v4.4s[0]
	fmla  v17.4s, v0.4s, v4.4s[1]
	fmla  v18.4s, v0.4s, v4.4s[2]
	fmla  v19.4s, v0.4s, v4.4s[3]

	sub w0, w0, #1

	fmla  v20.4s, v2.4s, v4.4s[0]
	fmla  v21.4s, v2.4s, v4.4s[1]
	fmla  v22.4s, v2.4s, v4.4s[2]
	fmla  v23.4s, v2.4s, v4.4s[3]

	cmp	w0, #0

	bgt	.L01_clean_loop



.L01_end_matter:

	cmp w8, #0

	beq .L01_store

// load_n C
	ld1 {v0.2d, v1.2d, v2.2d, v3.2d}, [x4], #64

	ld1 {v4.2d, v5.2d, v6.2d, v7.2d}, [x12], #64

	cmp w8, #0

	blt .L01_sub

	fadd v16.4s, v0.4s, v16.4s
	fadd v17.4s, v1.4s, v17.4s
	fadd v18.4s, v2.4s, v18.4s
	fadd v19.4s, v3.4s, v19.4s

	fadd v20.4s, v4.4s, v20.4s
	fadd v21.4s, v5.4s, v21.4s
	fadd v22.4s, v6.4s, v22.4s
	fadd v23.4s, v7.4s, v23.4s

	b .L01_store

.L01_sub:

	fsub v16.4s, v0.4s, v16.4s
	fsub v17.4s, v1.4s, v17.4s
	fsub v18.4s, v2.4s, v18.4s
	fsub v19.4s, v3.4s, v19.4s

	fsub v20.4s, v4.4s, v20.4s
	fsub v21.4s, v5.4s, v21.4s
	fsub v22.4s, v6.4s, v22.4s
	fsub v23.4s, v7.4s, v23.4s

.L01_store:

// store_n D
	st1 {v16.2d, v17.2d, v18.2d, v19.2d}, [x6], #64
	st1 {v20.2d, v21.2d, v22.2d, v23.2d}, [x13], #64

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







//                                    w0        x1        w2       x3        x4        w5       x6        w7       sp+0     sp+8    sp+16
// void kernel_sgemm_nt_12x4_lib4_new(int kmax, float *A, int sda, float *B, float *C, int sdc, float *D, int sdd, int alg, int tc, int td)

	.align	4
	.global	kernel_sgemm_nt_12x4_lib4_new
	.type	kernel_sgemm_nt_12x4_lib4_new, %function

kernel_sgemm_nt_12x4_lib4_new:

//	prfm  PLDL1KEEP, [x3, #0]
//	prfm  PLDL1KEEP, [x1, #0]

// load arguments from stack
	ldr w8, [sp, #0]
	ldr w9, [sp, #8]
	ldr w10, [sp, #16]

//	prfm  PLDL1KEEP, [x3, #64]
//	prfm  PLDL1KEEP, [x1, #64]

	lsl w2, w2, #4 // sda * bs * 4
	lsl w5, w5, #4 // sdc * bs * 4
	lsl w7, w7, #4 // sdd * bs * 4

	add x11, x1, x2 // A1
	add x12, x4, x5 // C1
	add x13, x6, x7 // D1

	add x14, x11, x2 // A2
	add x15, x12, x5 // C2
	add x16, x13, x7 // D2

//	prfm  PLDL1KEEP, [x11, #0]
//	prfm  PLDL1KEEP, [x11, #64]
	
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


// zero accumulation registers (upper 64-bit are zeroed automatically)
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

//	fmov    d28, d16
//	fmov    d29, d16
//	fmov    d30, d16
//	fmov    d31, d16

// A0_0
// v0.4s[0],  v0.4s[1],  v0.4s[2],  v0.4s[3]
//
// A1_0
// v2.4s[0],  v2.4s[1],  v2.4s[2],  v2.4s[3]
//
// A2_0
// v4.4s[0],  v4.4s[1],  v4.4s[2],  v4.4s[3]
//
// B_0
// v6.4s[0],  v6.4s[1],  v6.4s[2],  v6.4s[3]
//
// C0
// v16.4s[0], v17.4s[0], v18.4s[0], v19.4s[0]
// v16.4s[1], v17.4s[1], v18.4s[1], v19.4s[1]
// v16.4s[2], v17.4s[0], v18.4s[0], v19.4s[0]
// v16.4s[3], v17.4s[1], v18.4s[1], v19.4s[1]
//
// C1
// v20.4s[0], v21.4s[0], v22.4s[0], v23.4s[0]
// v20.4s[1], v21.4s[1], v22.4s[1], v23.4s[1]
// v20.4s[2], v21.4s[0], v22.4s[0], v23.4s[0]
// v20.4s[3], v21.4s[1], v22.4s[1], v23.4s[1]
//
// C2
// v24.4s[0], v25.4s[0], v26.4s[0], v27.4s[0]
// v24.4s[1], v25.4s[1], v26.4s[1], v27.4s[1]
// v24.4s[2], v25.4s[0], v26.4s[0], v27.4s[0]
// v24.4s[3], v25.4s[1], v26.4s[1], v27.4s[1]


	ble	.L02_end_matter


	cmp	w0, #3

	// prefetch
	ld1   {v0.4s, v1.4s}, [x1], #32
	ld1   {v6.4s, v7.4s}, [x3], #32
	ld1   {v2.4s, v3.4s}, [x11], #32
	ld1   {v4.4s, v5.4s}, [x14], #32

	ble	.L02_check_clean_loop2


.L02_main_loop:

	// 0
	fmla  v16.4s, v0.4s, v6.4s[0]
	ld1   {v8.4s, v9.4s}, [x1], #32
	fmla  v17.4s, v0.4s, v6.4s[1]
	ld1   {v14.4s, v15.4s}, [x3], #32
	fmla  v18.4s, v0.4s, v6.4s[2]
	ld1   {v10.4s, v11.4s}, [x11], #32
	fmla  v19.4s, v0.4s, v6.4s[3]
	ld1   {v12.4s, v13.4s}, [x14], #32

	fmla  v20.4s, v2.4s, v6.4s[0]
	prfm  PLDL1KEEP, [x1, #64]
	fmla  v21.4s, v2.4s, v6.4s[1]
	prfm  PLDL1KEEP, [x3, #64]
	fmla  v22.4s, v2.4s, v6.4s[2]
	prfm  PLDL1KEEP, [x11, #64]
	fmla  v23.4s, v2.4s, v6.4s[3]
	prfm  PLDL1KEEP, [x14, #64]

	fmla  v24.4s, v4.4s, v6.4s[0]
	fmla  v25.4s, v4.4s, v6.4s[1]
	fmla  v26.4s, v4.4s, v6.4s[2]
	fmla  v27.4s, v4.4s, v6.4s[3]

	sub w0, w0, #4

	// 1
	fmla  v16.4s, v1.4s, v7.4s[0]
	fmla  v17.4s, v1.4s, v7.4s[1]
	fmla  v18.4s, v1.4s, v7.4s[2]
	fmla  v19.4s, v1.4s, v7.4s[3]

	fmla  v20.4s, v3.4s, v7.4s[0]
	fmla  v21.4s, v3.4s, v7.4s[1]
	fmla  v22.4s, v3.4s, v7.4s[2]
	fmla  v23.4s, v3.4s, v7.4s[3]

	fmla  v24.4s, v5.4s, v7.4s[0]
	fmla  v25.4s, v5.4s, v7.4s[1]
	fmla  v26.4s, v5.4s, v7.4s[2]
	fmla  v27.4s, v5.4s, v7.4s[3]


	// 2
	fmla  v16.4s, v8.4s, v14.4s[0]
	ld1   {v0.4s, v1.4s}, [x1], #32
	fmla  v17.4s, v8.4s, v14.4s[1]
	ld1   {v6.4s, v7.4s}, [x3], #32
	fmla  v18.4s, v8.4s, v14.4s[2]
	ld1   {v2.4s, v3.4s}, [x11], #32
	fmla  v19.4s, v8.4s, v14.4s[3]
	ld1   {v4.4s, v5.4s}, [x14], #32

	fmla  v20.4s, v10.4s, v14.4s[0]
	fmla  v21.4s, v10.4s, v14.4s[1]
	fmla  v22.4s, v10.4s, v14.4s[2]
	fmla  v23.4s, v10.4s, v14.4s[3]

	fmla  v24.4s, v12.4s, v14.4s[0]
	fmla  v25.4s, v12.4s, v14.4s[1]
	fmla  v26.4s, v12.4s, v14.4s[2]
	fmla  v27.4s, v12.4s, v14.4s[3]


	cmp	w0, #3

	// 3
	fmla  v16.4s, v9.4s, v15.4s[0]
	fmla  v17.4s, v9.4s, v15.4s[1]
	fmla  v18.4s, v9.4s, v15.4s[2]
	fmla  v19.4s, v9.4s, v15.4s[3]

	fmla  v20.4s, v11.4s, v15.4s[0]
	fmla  v21.4s, v11.4s, v15.4s[1]
	fmla  v22.4s, v11.4s, v15.4s[2]
	fmla  v23.4s, v11.4s, v15.4s[3]

	fmla  v24.4s, v13.4s, v15.4s[0]
	fmla  v25.4s, v13.4s, v15.4s[1]
	fmla  v26.4s, v13.4s, v15.4s[2]
	fmla  v27.4s, v13.4s, v15.4s[3]


	bgt	.L02_main_loop



.L02_check_clean_loop2:

	cmp	w0, #1

	blt	.L02_end_matter

	beq	.L02_check_clean_loop1


	// 0
	fmla  v16.4s, v0.4s, v6.4s[0]
	fmla  v17.4s, v0.4s, v6.4s[1]
	fmla  v18.4s, v0.4s, v6.4s[2]
	fmla  v19.4s, v0.4s, v6.4s[3]

	fmla  v20.4s, v2.4s, v6.4s[0]
	fmla  v21.4s, v2.4s, v6.4s[1]
	fmla  v22.4s, v2.4s, v6.4s[2]
	fmla  v23.4s, v2.4s, v6.4s[3]

	fmla  v24.4s, v4.4s, v6.4s[0]
	fmla  v25.4s, v4.4s, v6.4s[1]
	fmla  v26.4s, v4.4s, v6.4s[2]
	fmla  v27.4s, v4.4s, v6.4s[3]

	sub w0, w0, #2

	// 1
	fmla  v16.4s, v1.4s, v7.4s[0]
	fmla  v17.4s, v1.4s, v7.4s[1]
	fmla  v18.4s, v1.4s, v7.4s[2]
	fmla  v19.4s, v1.4s, v7.4s[3]

	fmla  v20.4s, v3.4s, v7.4s[0]
	fmla  v21.4s, v3.4s, v7.4s[1]
	fmla  v22.4s, v3.4s, v7.4s[2]
	fmla  v23.4s, v3.4s, v7.4s[3]

	cmp	w0, #0

	fmla  v24.4s, v5.4s, v7.4s[0]
	fmla  v25.4s, v5.4s, v7.4s[1]
	fmla  v26.4s, v5.4s, v7.4s[2]
	fmla  v27.4s, v5.4s, v7.4s[3]

	ble	.L02_end_matter

	ld1   {v0.4s}, [x1], #16
	ld1   {v6.4s}, [x3], #16
	ld1   {v2.4s}, [x11], #16
	ld1   {v4.4s}, [x14], #16

.L02_check_clean_loop1:

	// 0
	fmla  v16.4s, v0.4s, v6.4s[0]
	fmla  v17.4s, v0.4s, v6.4s[1]
	fmla  v18.4s, v0.4s, v6.4s[2]
	fmla  v19.4s, v0.4s, v6.4s[3]

	fmla  v20.4s, v2.4s, v6.4s[0]
	fmla  v21.4s, v2.4s, v6.4s[1]
	fmla  v22.4s, v2.4s, v6.4s[2]
	fmla  v23.4s, v2.4s, v6.4s[3]

	fmla  v24.4s, v4.4s, v6.4s[0]
	fmla  v25.4s, v4.4s, v6.4s[1]
	fmla  v26.4s, v4.4s, v6.4s[2]
	fmla  v27.4s, v4.4s, v6.4s[3]


.L02_end_matter:

	cmp w8, #0

	beq .L02_store

// load_n C
	ld1 {v0.2d, v1.2d, v2.2d, v3.2d}, [x4], #64

	ld1 {v4.2d, v5.2d, v6.2d, v7.2d}, [x12], #64

	ld1 {v8.2d, v9.2d, v10.2d, v11.2d}, [x15], #64

	cmp w8, #0

	blt .L02_sub

	fadd v16.4s, v0.4s, v16.4s
	fadd v17.4s, v1.4s, v17.4s
	fadd v18.4s, v2.4s, v18.4s
	fadd v19.4s, v3.4s, v19.4s

	fadd v20.4s, v4.4s, v20.4s
	fadd v21.4s, v5.4s, v21.4s
	fadd v22.4s, v6.4s, v22.4s
	fadd v23.4s, v7.4s, v23.4s

	fadd v24.4s, v8.4s, v24.4s
	fadd v25.4s, v9.4s, v25.4s
	fadd v26.4s, v10.4s, v26.4s
	fadd v27.4s, v11.4s, v27.4s

	b .L02_store

.L02_sub:

	fsub v16.4s, v0.4s, v16.4s
	fsub v17.4s, v1.4s, v17.4s
	fsub v18.4s, v2.4s, v18.4s
	fsub v19.4s, v3.4s, v19.4s

	fsub v20.4s, v4.4s, v20.4s
	fsub v21.4s, v5.4s, v21.4s
	fsub v22.4s, v6.4s, v22.4s
	fsub v23.4s, v7.4s, v23.4s

	fsub v24.4s, v8.4s, v24.4s
	fsub v25.4s, v9.4s, v25.4s
	fsub v26.4s, v10.4s, v26.4s
	fsub v27.4s, v11.4s, v27.4s

.L02_store:

// store_n D
	st1 {v16.2d, v17.2d, v18.2d, v19.2d}, [x6], #64

	st1 {v20.2d, v21.2d, v22.2d, v23.2d}, [x13], #64

	st1 {v24.2d, v25.2d, v26.2d, v27.2d}, [x16], #64

.L02_epilogue:
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



