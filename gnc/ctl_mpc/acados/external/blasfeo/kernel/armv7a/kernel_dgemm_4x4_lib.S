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
// triangular substitution:
// side = right
// uplo = lower
// tran = transposed
// requires explicit inverse of diagonal
//
// input arguments:
// r4   <- E
// r5   <- lde*sizeof(double)
// r6   <- inv_diag_E
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLT_INV_4X4_LIB
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_dtrsm_rlt_inv_4x4_lib, %function
inner_edge_dtrsm_rlt_inv_4x4_lib:
#elif defined(OS_MAC)
inner_edge_dtrsm_rlt_inv_4x4_lib:
#endif
#endif
	
	//
	fldd		d16, [r6, #0] // E_inv[0]
	fmuld		d0, d0, d16
	fmuld		d1, d1, d16
	fmuld		d2, d2, d16
	fmuld		d3, d3, d16
	fldd		d16, [r4, #8] // E[1+4*0]
	fnmacd		d4, d0, d16
	fnmacd		d5, d1, d16
	fnmacd		d6, d2, d16
	fnmacd		d7, d3, d16
	fldd		d16, [r4, #16] // E[2+4*0]
	fnmacd		d8, d0, d16
	fnmacd		d9, d1, d16
	fnmacd		d10, d2, d16
	fnmacd		d11, d3, d16
	fldd		d16, [r4, #24] // E[3+4*0]
	fnmacd		d12, d0, d16
	fnmacd		d13, d1, d16
	fnmacd		d14, d2, d16
	fnmacd		d15, d3, d16

	//
	add			r4, r4, r5
	fldd		d16, [r6, #8] // E_inv[1]
	fmuld		d4, d4, d16
	fmuld		d5, d5, d16
	fmuld		d6, d6, d16
	fmuld		d7, d7, d16
	fldd		d16, [r4, #16] // E[2+4*1]
	fnmacd		d8, d4, d16
	fnmacd		d9, d5, d16
	fnmacd		d10, d6, d16
	fnmacd		d11, d7, d16
	fldd		d16, [r4, #24] // E[3+4*1]
	fnmacd		d12, d4, d16
	fnmacd		d13, d5, d16
	fnmacd		d14, d6, d16
	fnmacd		d15, d7, d16

	//
	add			r4, r4, r5
	fldd		d16, [r6, #16] // E_inv[2]
	fmuld		d8, d8, d16
	fmuld		d9, d9, d16
	fmuld		d10, d10, d16
	fmuld		d11, d11, d16
	fldd		d16, [r4, #24] // E[3+4*2]
	fnmacd		d12, d8, d16
	fnmacd		d13, d9, d16
	fnmacd		d14, d10, d16
	fnmacd		d15, d11, d16

	//
//	add			r4, r4, r5
	fldd		d16, [r6, #24] // E_inv[3]
	fmuld		d12, d12, d16
	fmuld		d13, d13, d16
	fmuld		d14, d14, d16
	fmuld		d15, d15, d16
	
#if MACRO_LEVEL>=1
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_edge_dtrsm_rlt_inv_4x4_lib, .-inner_edge_dtrsm_rlt_inv_4x4_lib
#endif
#endif





// subroutine
//
// input arguments:
// r4   <- alpha
// r5   <- beta
// r6   <- C
// r7   <- ldc*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_4X4_LIB lc_zero
#else
	.align 3
99: // 0
	.word 0
	.word 0

	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_ab_4x4_lib, %function
inner_scale_ab_4x4_lib:
#elif defined(OS_MAC)
_inner_scale_ab_4x4_lib:
#endif
#endif

	fldd	d16, [r4, #0] // alpha

	fmuld	d0, d0, d16
	fldd	d18, [r5, #0] // beta
	fmuld	d1, d1, d16
#if MACRO_LEVEL>=1
	fldd	d17, \lc_zero // 0.0
#else
	fldd	d17, 99b // 0.0
#endif
	fmuld	d2, d2, d16
	fmuld	d3, d3, d16

	fmuld	d4, d4, d16
	fmuld	d5, d5, d16
	fmuld	d6, d6, d16
	fmuld	d7, d7, d16

	fmuld	d8, d8, d16
	fcmped	d18, d17
	fmuld	d9, d9, d16
	fmuld	d10, d10, d16
	fmuld	d11, d11, d16

	fmuld	d12, d12, d16
	fmstat
	fmuld	d13, d13, d16
	fmuld	d14, d14, d16
	fmuld	d15, d15, d16

	beq		0f // end

	fldd	d17, [r6, #0] // C
	fmacd	d0, d18, d17
	fldd	d17, [r6, #8] // C
	fmacd	d1, d18, d17
	fldd	d17, [r6, #16] // C
	fmacd	d2, d18, d17
	fldd	d17, [r6, #24] // C
	fmacd	d3, d18, d17

	add		r6, r6, r7
	fldd	d17, [r6, #0] // C
	fmacd	d4, d18, d17
	fldd	d17, [r6, #8] // C
	fmacd	d5, d18, d17
	fldd	d17, [r6, #16] // C
	fmacd	d6, d18, d17
	fldd	d17, [r6, #24] // C
	fmacd	d7, d18, d17

	add		r6, r6, r7
	fldd	d17, [r6, #0] // C
	fmacd	d8, d18, d17
	fldd	d17, [r6, #8] // C
	fmacd	d9, d18, d17
	fldd	d17, [r6, #16] // C
	fmacd	d10, d18, d17
	fldd	d17, [r6, #24] // C
	fmacd	d11, d18, d17

	add		r6, r6, r7
	fldd	d17, [r6, #0] // C
	fmacd	d12, d18, d17
	fldd	d17, [r6, #8] // C
	fmacd	d13, d18, d17
	fldd	d17, [r6, #16] // C
	fmacd	d14, d18, d17
	fldd	d17, [r6, #24] // C
	fmacd	d15, d18, d17

0:

#if MACRO_LEVEL>=1
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_scale_ab_4x4_lib, .-inner_scale_ab_4x4_lib
#endif
#endif





// subroutine
//
// input arguments:
// r4   <- C
// r5   <- ldc*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M11_4X4_LIB
#else
//	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_m11_4x4_lib, %function
inner_scale_m11_4x4_lib:
#elif defined(OS_MAC)
_inner_scale_m11_4x4_lib:
#endif
#endif

	fldd	d17, [r4, #0] // C
	fsubd	d0, d17, d0
	fldd	d17, [r4, #8] // C
	fsubd	d1, d17, d1
	fldd	d17, [r4, #16] // C
	fsubd	d2, d17, d2
	fldd	d17, [r4, #24] // C
	fsubd	d3, d17, d3

	add		r4, r4, r5
	fldd	d17, [r4, #0] // C
	fsubd	d4, d17, d4
	fldd	d17, [r4, #8] // C
	fsubd	d5, d17, d5
	fldd	d17, [r4, #16] // C
	fsubd	d6, d17, d6
	fldd	d17, [r4, #24] // C
	fsubd	d7, d17, d7

	add		r4, r4, r5
	fldd	d17, [r4, #0] // C
	fsubd	d8, d17, d8
	fldd	d17, [r4, #8] // C
	fsubd	d9, d17, d9
	fldd	d17, [r4, #16] // C
	fsubd	d10, d17, d10
	fldd	d17, [r4, #24] // C
	fsubd	d11, d17, d11

	add		r4, r4, r5
	fldd	d17, [r4, #0] // C
	fsubd	d12, d17, d12
	fldd	d17, [r4, #8] // C
	fsubd	d13, d17, d13
	fldd	d17, [r4, #16] // C
	fsubd	d14, d17, d14
	fldd	d17, [r4, #24] // C
	fsubd	d15, d17, d15


#if MACRO_LEVEL>=1
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_scale_m11_4x4_lib, .-inner_scale_m11_4x4_lib
#endif
#endif





// subroutine
//
// input arguments:
// r4   <- beta
// r5   <- C
// r6   <- ldc*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M1B_4X4_LIB lc_zero
#else
	.align 3
99: // 0
	.word 0
	.word 0

	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_m1b_4x4_lib, %function
inner_scale_m1b_4x4_lib:
#elif defined(OS_MAC)
_inner_scale_m1b_4x4_lib:
#endif
#endif

	fldd	d18, [r4, #0] // beta
#if MACRO_LEVEL>=1
	fldd	d17, \lc_zero // 0.0
#else
	fldd	d17, 99b // 0.0
#endif

	fcmped	d18, d17
	fmstat

	beq		0f // end

	fldd	d17, [r5, #0] // C
	fmscd	d0, d18, d17
	fldd	d17, [r5, #8] // C
	fmscd	d1, d18, d17
	fldd	d17, [r5, #16] // C
	fmscd	d2, d18, d17
	fldd	d17, [r5, #24] // C
	fmscd	d3, d18, d17

	add		r5, r5, r6
	fldd	d17, [r5, #0] // C
	fmscd	d4, d18, d17
	fldd	d17, [r5, #8] // C
	fmscd	d5, d18, d17
	fldd	d17, [r5, #16] // C
	fmscd	d6, d18, d17
	fldd	d17, [r5, #24] // C
	fmscd	d7, d18, d17

	add		r5, r5, r6
	fldd	d17, [r5, #0] // C
	fmscd	d8, d18, d17
	fldd	d17, [r5, #8] // C
	fmscd	d9, d18, d17
	fldd	d17, [r5, #16] // C
	fmscd	d10, d18, d17
	fldd	d17, [r5, #24] // C
	fmscd	d11, d18, d17

	add		r5, r5, r6
	fldd	d17, [r5, #0] // C
	fmscd	d12, d18, d17
	fldd	d17, [r5, #8] // C
	fmscd	d13, d18, d17
	fldd	d17, [r5, #16] // C
	fmscd	d14, d18, d17
	fldd	d17, [r5, #24] // C
	fmscd	d15, d18, d17

0:

#if MACRO_LEVEL>=1
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_scale_m1b_4x4_lib, .-inner_scale_m1b_4x4_lib
#endif
#endif





// subroutine
//
// input arguments:
// r4   <- D
// r5   <- ldd*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X4_LIB
#else
//	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_4x4_lib, %function
inner_store_4x4_lib:
#elif defined(OS_MAC)
_inner_store_4x4_lib:
#endif
#endif

	fstd	d0, [r4, #0]
	fstd	d1, [r4, #8]
	fstd	d2, [r4, #16]
	fstd	d3, [r4, #24]

	add		r4, r4, r5
	fstd	d4, [r4, #0]
	fstd	d5, [r4, #8]
	fstd	d6, [r4, #16]
	fstd	d7, [r4, #24]

	add		r4, r4, r5
	fstd	d8, [r4, #0]
	fstd	d9, [r4, #8]
	fstd	d10, [r4, #16]
	fstd	d11, [r4, #24]

	add		r4, r4, r5
	fstd	d12, [r4, #0]
	fstd	d13, [r4, #8]
	fstd	d14, [r4, #16]
	fstd	d15, [r4, #24]

#if MACRO_LEVEL>=1
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_store_4x4_lib, .-inner_store_4x4_lib
#endif
#endif





// subroutine
//
// input arguments:
// r4   <- D
// r5   <- ldd*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_4X4_LIB
#else
//	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_l_4x4_lib, %function
inner_store_l_4x4_lib:
#elif defined(OS_MAC)
_inner_store_l_4x4_lib4:
#endif
#endif

	fstd	d0, [r4, #0]
	fstd	d1, [r4, #8]
	fstd	d2, [r4, #16]
	fstd	d3, [r4, #24]

	add		r4, r4, r5
//	fstd	d4, [r4, #0]
	fstd	d5, [r4, #8]
	fstd	d6, [r4, #16]
	fstd	d7, [r4, #24]

	add		r4, r4, r5
//	fstd	d8, [r4, #0]
//	fstd	d9, [r4, #8]
	fstd	d10, [r4, #16]
	fstd	d11, [r4, #24]

	add		r4, r4, r5
//	fstd	d12, [r4, #0]
//	fstd	d13, [r4, #8]
//	fstd	d14, [r4, #16]
	fstd	d15, [r4, #24]

#if MACRO_LEVEL>=1
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_store_l_4x4_lib, .-inner_store_l_4x4_lib
#endif
#endif





	.align 3
99: // 0
	.word 0
	.word 0




//                                 r0        r1             r2         r3         sp+0          sp+4       sp+8     sp+12      sp+16
// void kernel_dgemm_nt_4x4_lib44cc(int kmax, double *alpha, double *A, double *B, double *beta, double *C, int ldc, double *D, int ldd)

//	.p2align 4,,15
#if defined(OS_LINUX)
	.global	kernel_dgemm_nt_4x4_lib44cc
	.type	kernel_dgemm_nt_4x4_lib44cc, %function
kernel_dgemm_nt_4x4_lib44cc:
#elif defined(OS_MAC)
	.global	kernel_dgemm_nt_4x4_lib44cc
_kernel_dgemm_nt_4x4_lib44cc:
#endif

	// prologue

	// save GP registers
	stmdb	sp!, {r4 - r10, fp, lr} // save registers
	add		fp, sp, #36 // fp to old sp position

	// save FP registers
	fstmfdd	sp!, {d8-d15}



	// zero accumulation registers
	fldd	d0, 99b
	fcpyd	d1, d0
	fcpyd	d2, d0
	fcpyd	d3, d0
	fcpyd	d4, d0
	fcpyd	d5, d0
	fcpyd	d6, d0
	fcpyd	d7, d0
	fcpyd	d8, d0
	fcpyd	d9, d0
	fcpyd	d10, d0
	fcpyd	d11, d0
	fcpyd	d12, d0
	fcpyd	d13, d0
	fcpyd	d14, d0
	fcpyd	d15, d0



	// call inner kernel dgemm nt
	mov		r4, r0 // kmax
	mov		r5, r2 // A
	mov		r6, r3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_ADD_NT_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl	inner_kernel_dgemm_add_nt_4x4_lib4
#elif defined(OS_MAC)
	bl	_inner_kernel_dgemm_add_nt_4x4_lib4
#endif
#endif



	// call inner blend for generic alpha and beta
	mov		r4, r1 // alpha
	ldr		r5, [fp, #0] // beta
	ldr		r6, [fp, #4] // C
	ldr		r7, [fp, #8] // ldc
	lsl		r7, r7, #3 // ldc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB 99f
#else
#if defined(OS_LINUX)
	bl inner_scale_ab_4x4_lib
#elif defined(OS_MAC)
	bl _inner_scale_ab_4x4_lib
#endif
#endif



	// store n
	ldr		r4, [fp, #12] // D
	ldr		r5, [fp, #16] // ldd
	lsl		r5, r5, #3 // ldd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
#if defined(OS_LINUX)
	bl inner_store_4x4_lib
#elif defined(OS_MAC)
	bl _inner_store_4x4_lib
#endif
#endif



	// epilogue

	// load FP registers
	fldmfdd	sp!, {d8-d15}

	// load GP registers and return
//	ldmia	sp!, {r4 - r10, fp, lr} // load registers
//	mov		pc, lr // return
	ldmia	sp!, {r4 - r10, fp, pc} // load registers and return

#if defined(OS_LINUX)
	.size	kernel_dgemm_nt_4x4_lib44cc, .-kernel_dgemm_nt_4x4_lib44cc
#endif





	.align 3
99: // 0
	.word 0
	.word 0





//                                    r0        r1         r2         r3         sp+0     sp+4       sp+8     sp+12
// void kernel_dpotrf_nt_l_4x4_lib44cc(int kmax, double *A, double *B, double *C, int ldc, double *D, int ldd, double *inv_diag_D);

//	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dpotrf_nt_l_4x4_lib44cc
	.type kernel_dpotrf_nt_l_4x4_lib44cc, %function
kernel_dpotrf_nt_l_4x4_lib44cc:
#elif defined(OS_MAC)
	.globl _kernel_dpotrf_nt_l_4x4_lib44cc
_kernel_dpotrf_nt_l_4x4_lib44cc:
#endif

	// prologue

	// save GP registers
	stmdb	sp!, {r4 - r10, fp, lr} // save registers
	add		fp, sp, #36 // fp to old sp position

	// save FP registers
	fstmfdd	sp!, {d8-d15}



	// zero accumulation registers
	fldd	d0, 99b
	fcpyd	d1, d0
	fcpyd	d2, d0
	fcpyd	d3, d0
	fcpyd	d4, d0
	fcpyd	d5, d0
	fcpyd	d6, d0
	fcpyd	d7, d0
	fcpyd	d8, d0
	fcpyd	d9, d0
	fcpyd	d10, d0
	fcpyd	d11, d0
	fcpyd	d12, d0
	fcpyd	d13, d0
	fcpyd	d14, d0
	fcpyd	d15, d0



	// call inner kernel dsyrk l nt
	mov		r4, r0 // kmax
	mov		r5, r1 // A
	mov		r6, r2 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DSYRK_L_ADD_NT_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl	inner_kernel_dsyrk_l_add_nt_4x4_lib4
#elif defined(OS_MAC)
	bl	_inner_kernel_dsyrk_l_add_nt_4x4_lib4
#endif
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		r4, r3 // C
	ldr		r5, [fp, #0] // ldc
	lsl		r5, r5, #3 // ldc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X4_LIB
#else
#if defined(OS_LINUX)
	bl inner_scale_m11_4x4_lib
#elif defined(OS_MAC)
	bl _inner_scale_m11_4x4_lib
#endif
#endif



	// factorization
	ldr		r4, [fp, #12] // inv_diag_D

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_4X4_LIB4 99f
#else
#if defined(OS_LINUX)
	bl inner_edge_dpotrf_4x4_lib4
#elif defined(OS_MAC)
	bl _inner_edge_dpotrf_4x4_lib4
#endif
#endif



	// store l
	ldr		r4, [fp, #4] // D
	ldr		r5, [fp, #8] // ldd
	lsl		r5, r5, #3 // ldd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_LIB
#else
#if defined(OS_LINUX)
	bl inner_store_l_4x4_lib
#elif defined(OS_MAC)
	bl _inner_store_l_4x4_lib
#endif
#endif



	// epilogue

	// load FP registers
	fldmfdd	sp!, {d8-d15}

	// load GP registers and return
//	ldmia	sp!, {r4 - r10, fp, lr} // load registers
//	mov		pc, lr // return
	ldmia	sp!, {r4 - r10, fp, pc} // load registers and return

#if defined(OS_LINUX)
	.size	kernel_dpotrf_nt_l_4x4_lib44cc, .-kernel_dpotrf_nt_l_4x4_lib44cc
#endif





	.align 3
99: // 0
	.word 0
	.word 0




//                                          r0        r1         r2         r3            sp+0       sp+4     rsp+8      rsp+12   rsp+16     rsp+20   rsp+24
// void kernel_dtrsm_nt_rl_inv_4x4_lib44ccc(int kmax, double *A, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, double *inv_diag_E);

//	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dtrsm_nt_rl_inv_4x4_lib44ccc
	.type kernel_dtrsm_nt_rl_inv_4x4_lib44ccc, %function
kernel_dtrsm_nt_rl_inv_4x4_lib44ccc:
#elif defined(OS_MAC)
	.globl _kernel_dtrsm_nt_rl_inv_4x4_lib44ccc
_kernel_dtrsm_nt_rl_inv_4x4_lib44ccc:
#endif

	// prologue

	// save GP registers
	stmdb	sp!, {r4 - r10, fp, lr} // save registers
	add		fp, sp, #36 // fp to old sp position

	// save FP registers
	fstmfdd	sp!, {d8-d15}



	// zero accumulation registers
	fldd	d0, 99b
	fcpyd	d1, d0
	fcpyd	d2, d0
	fcpyd	d3, d0
	fcpyd	d4, d0
	fcpyd	d5, d0
	fcpyd	d6, d0
	fcpyd	d7, d0
	fcpyd	d8, d0
	fcpyd	d9, d0
	fcpyd	d10, d0
	fcpyd	d11, d0
	fcpyd	d12, d0
	fcpyd	d13, d0
	fcpyd	d14, d0
	fcpyd	d15, d0



	// call inner kernel dgemm nt
	mov		r4, r0 // kmax
	mov		r5, r1 // A
	mov		r6, r2 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_ADD_NT_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl	inner_kernel_dgemm_add_nt_4x4_lib4
#elif defined(OS_MAC)
	bl	_inner_kernel_dgemm_add_nt_4x4_lib4
#endif
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		r4, r3 // beta
	ldr		r5, [fp, #0] // C
	ldr		r6, [fp, #4] // ldc
	lsl		r6, r6, #3 // ldc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB 99f
#else
#if defined(OS_LINUX)
	bl inner_scale_m1b_4x4_lib
#elif defined(OS_MAC)
	bl _inner_scale_m1b_4x4_lib
#endif
#endif



	// solution
	ldr		r4, [fp, #16] // E
	ldr		r5, [fp, #20] // lde
	lsl		r5, r5, #3 // lde*sizeof(double)
	ldr		r6, [fp, #24] // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_4X4_LIB
#else
#if defined(OS_LINUX)
	bl inner_edge_dtrsm_rlt_inv_4x4_lib
#elif defined(OS_MAC)
	bl _inner_edge_dtrsm_rlt_inv_4x4_lib
#endif
#endif



	// store
	ldr		r4, [fp, #8] // D
	ldr		r5, [fp, #12] // ldd
	lsl		r5, r5, #3 // ldd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
#if defined(OS_LINUX)
	bl inner_store_4x4_lib
#elif defined(OS_MAC)
	bl _inner_store_4x4_lib
#endif
#endif



	// epilogue

	// load FP registers
	fldmfdd	sp!, {d8-d15}

	// load GP registers and return
//	ldmia	sp!, {r4 - r10, fp, lr} // load registers
//	mov		pc, lr // return
	ldmia	sp!, {r4 - r10, fp, pc} // load registers and return

#if defined(OS_LINUX)
	.size	kernel_dtrsm_nt_rl_inv_4x4_lib44ccc, .-kernel_dtrsm_nt_rl_inv_4x4_lib44ccc
#endif





	.align 3
99: // 0
	.word 0
	.word 0




