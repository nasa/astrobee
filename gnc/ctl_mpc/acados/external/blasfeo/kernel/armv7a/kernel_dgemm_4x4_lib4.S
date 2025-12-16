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
	.text
#elif defined(OS_MAC)
	.section	__TEXT,__text,regular,pure_instructions
#endif



// subroutine
//
// input arguments:
// r4   <- k
// r5   <- A
// r6   <- B
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_ADD_NT_4X4_LIB4
#else
//	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_dgemm_add_nt_4x4_lib4, %function
inner_kernel_dgemm_add_nt_4x4_lib4:
#elif defined(OS_MAC)
_inner_kernel_dgemm_add_nt_4x4_lib4:
#endif
#endif

	// early return
	cmp		r4, #0
	ble		2f // return

	// prefetch
	pld		[r5, #0]
	pld		[r6, #0]
#if defined(TARGET_ARMV7A_ARM_CORTEX_A9)
	pld		[r5, #32]
	pld		[r6, #32]
#endif

	// preload A even
	fldd	d16, [r5, #0]
	fldd	d17, [r5, #8]
	fldd	d18, [r5, #16]
	fldd	d19, [r5, #24]

	// preload B even
	fldd	d20, [r6, #0]
	fldd	d21, [r6, #8]
	fldd	d22, [r6, #16]
	fldd	d23, [r6, #24]

	// preload A odd
	fldd	d24, [r5, #32]
	fldd	d25, [r5, #40]
	fldd	d26, [r5, #48]
	fldd	d27, [r5, #56]

	// preload B odd
	fldd	d28, [r6, #32]
	fldd	d29, [r6, #40]
	fldd	d30, [r6, #48]
	fldd	d31, [r6, #56]

	// prefetch
	pld		[r5, #64]
	pld		[r6, #64]
#if defined(TARGET_ARMV7A_ARM_CORTEX_A9)
	pld		[r5, #96]
	pld		[r6, #96]
#endif

	cmp		r4, #4
	ble		0f // consider clean up loop

	// main loop
1:
	
	// unroll 0
	fmacd	d0, d16, d20
	pld		[r5, #128] // prefetch
	fmacd	d1, d17, d20
	pld		[r6, #128] // prefetch
	fmacd	d2, d18, d20
	fmacd	d3, d19, d20
	fldd	d20, [r6, #64] // B

	fmacd	d4, d16, d21
	fmacd	d5, d17, d21
	fmacd	d6, d18, d21
	fmacd	d7, d19, d21
	fldd	d21, [r6, #72] // B

	fmacd	d8, d16, d22
	fmacd	d9, d17, d22
	fmacd	d10, d18, d22
	fmacd	d11, d19, d22
	fldd	d22, [r6, #80] // B

	fmacd	d12, d16, d23
	fldd	d16, [r5, #64] // A
	fmacd	d13, d17, d23
	fldd	d17, [r5, #72] // A
	fmacd	d14, d18, d23
	fldd	d18, [r5, #80] // A
	fmacd	d15, d19, d23
	fldd	d19, [r5, #88] // A
	fldd	d23, [r6, #88] // B

	// unroll 1
	fmacd	d0, d24, d28
#if defined(TARGET_ARMV7A_ARM_CORTEX_A9)
	pld		[r5, #160] // prefetch
#endif
	fmacd	d1, d25, d28
#if defined(TARGET_ARMV7A_ARM_CORTEX_A9)
	pld		[r6, #160] // prefetch
#endif
	fmacd	d2, d26, d28
	fmacd	d3, d27, d28
	fldd	d28, [r6, #96] // B

	fmacd	d4, d24, d29
	fmacd	d5, d25, d29
	fmacd	d6, d26, d29
	fmacd	d7, d27, d29
	fldd	d29, [r6, #104] // B

	fmacd	d8, d24, d30
	fmacd	d9, d25, d30
	fmacd	d10, d26, d30
	fmacd	d11, d27, d30
	fldd	d30, [r6, #112] // B

	fmacd	d12, d24, d31
	fldd	d24, [r5, #96] // A
	fmacd	d13, d25, d31
	fldd	d25, [r5, #104] // A
	fmacd	d14, d26, d31
	fldd	d26, [r5, #112] // A
	fmacd	d15, d27, d31
	fldd	d27, [r5, #120] // A
	fldd	d31, [r6, #120] // B

	// unroll 2
	fmacd	d0, d16, d20
	pld		[r6, #192] // prefetch
	fmacd	d1, d17, d20
	add		r6, r6, #128
	fmacd	d2, d18, d20
	fmacd	d3, d19, d20
	fldd	d20, [r6, #0] // B

	fmacd	d4, d16, d21
	pld		[r5, #192] // prefetch
	fmacd	d5, d17, d21
	add		r5, r5, #128
	fmacd	d6, d18, d21
	fmacd	d7, d19, d21
	fldd	d21, [r6, #8] // B

	fmacd	d8, d16, d22
	fmacd	d9, d17, d22
	fmacd	d10, d18, d22
	fmacd	d11, d19, d22
	fldd	d22, [r6, #16] // B

	fmacd	d12, d16, d23
	fldd	d16, [r5, #0] // A
	fmacd	d13, d17, d23
	fldd	d17, [r5, #8] // A
	fmacd	d14, d18, d23
	fldd	d18, [r5, #16] // A
	fmacd	d15, d19, d23
	fldd	d19, [r5, #24] // A
	fldd	d23, [r6, #24] // B

	// unroll 3
	fmacd	d0, d24, d28
#if defined(TARGET_ARMV7A_ARM_CORTEX_A9)
	pld		[r5, #96] // prefetch
#endif
	fmacd	d1, d25, d28
#if defined(TARGET_ARMV7A_ARM_CORTEX_A9)
	pld		[r6, #96] // prefetch
#endif
	fmacd	d2, d26, d28
	fmacd	d3, d27, d28
	fldd	d28, [r6, #32] // B

	fmacd	d4, d24, d29
	fmacd	d5, d25, d29
	fmacd	d6, d26, d29
	fmacd	d7, d27, d29
	fldd	d29, [r6, #40] // B

	fmacd	d8, d24, d30
	sub		r4, r4, #4
	fmacd	d9, d25, d30
	fmacd	d10, d26, d30
	fmacd	d11, d27, d30
	fldd	d30, [r6, #48] // B

	fmacd	d12, d24, d31
	fldd	d24, [r5, #32] // A
	fmacd	d13, d25, d31
	fldd	d25, [r5, #40] // A
	fmacd	d14, d26, d31
	fldd	d26, [r5, #48] // A
	fmacd	d15, d27, d31
	fldd	d27, [r5, #56] // A
	fldd	d31, [r6, #56] // B

	cmp		r4, #4
	bgt		1b

0:

	cmp		r4, #3
	ble		4f

	// unroll 0
	fmacd	d0, d16, d20
	fmacd	d1, d17, d20
	fmacd	d2, d18, d20
	fmacd	d3, d19, d20
	fldd	d20, [r6, #64] // B

	fmacd	d4, d16, d21
	fmacd	d5, d17, d21
	fmacd	d6, d18, d21
	fmacd	d7, d19, d21
	fldd	d21, [r6, #72] // B

	fmacd	d8, d16, d22
	fmacd	d9, d17, d22
	fmacd	d10, d18, d22
	fmacd	d11, d19, d22
	fldd	d22, [r6, #80] // B

	fmacd	d12, d16, d23
	fldd	d16, [r5, #64] // A
	fmacd	d13, d17, d23
	fldd	d17, [r5, #72] // A
	fmacd	d14, d18, d23
	fldd	d18, [r5, #80] // A
	fmacd	d15, d19, d23
	fldd	d19, [r5, #88] // A
	fldd	d23, [r6, #88] // B

	// unroll 1
	fmacd	d0, d24, d28
	fmacd	d1, d25, d28
	fmacd	d2, d26, d28
	fmacd	d3, d27, d28
	fldd	d28, [r6, #96] // B

	fmacd	d4, d24, d29
	fmacd	d5, d25, d29
	fmacd	d6, d26, d29
	fmacd	d7, d27, d29
	fldd	d29, [r6, #104] // B

	fmacd	d8, d24, d30
	fmacd	d9, d25, d30
	fmacd	d10, d26, d30
	fmacd	d11, d27, d30
	fldd	d30, [r6, #112] // B

	fmacd	d12, d24, d31
	fldd	d24, [r5, #96] // A
	fmacd	d13, d25, d31
	fldd	d25, [r5, #104] // A
	fmacd	d14, d26, d31
	fldd	d26, [r5, #112] // A
	fmacd	d15, d27, d31
	fldd	d27, [r5, #120] // A
	fldd	d31, [r6, #120] // B

	add		r5, r5, #128
	add		r6, r6, #128

	// unroll 2
	fmacd	d0, d16, d20
	fmacd	d1, d17, d20
	fmacd	d2, d18, d20
	fmacd	d3, d19, d20

	fmacd	d4, d16, d21
	fmacd	d5, d17, d21
	fmacd	d6, d18, d21
	fmacd	d7, d19, d21

	fmacd	d8, d16, d22
	fmacd	d9, d17, d22
	fmacd	d10, d18, d22
	fmacd	d11, d19, d22

	fmacd	d12, d16, d23
	fmacd	d13, d17, d23
	fmacd	d14, d18, d23
	fmacd	d15, d19, d23

	// unroll 3
	fmacd	d0, d24, d28
	fmacd	d1, d25, d28
	fmacd	d2, d26, d28
	fmacd	d3, d27, d28

	fmacd	d4, d24, d29
	fmacd	d5, d25, d29
	fmacd	d6, d26, d29
	fmacd	d7, d27, d29

	fmacd	d8, d24, d30
	fmacd	d9, d25, d30
	fmacd	d10, d26, d30
	fmacd	d11, d27, d30

	fmacd	d12, d24, d31
	fmacd	d13, d25, d31
	fmacd	d14, d26, d31
	fmacd	d15, d27, d31

	sub		r4, r4, #4

	b		2f // return

4: // consider clean1-up loop

	cmp		r4, #0
	ble		2f // return

3: // clean1-up loop

	fldd	d16, [r5, #0] // A
	fldd	d17, [r5, #8] // A
	fldd	d18, [r5, #16] // A
	fldd	d19, [r5, #24] // A

	fldd	d20, [r6, #0] // B
	fmacd	d0, d16, d20
	fmacd	d1, d17, d20
	fmacd	d2, d18, d20
	fmacd	d3, d19, d20

	fldd	d21, [r6, #8] // B
	fmacd	d4, d16, d21
	fmacd	d5, d17, d21
	fmacd	d6, d18, d21
	fmacd	d7, d19, d21

	fldd	d22, [r6, #16] // B
	fmacd	d8, d16, d22
	fmacd	d9, d17, d22
	fmacd	d10, d18, d22
	fmacd	d11, d19, d22

	fldd	d23, [r6, #24] // B
	fmacd	d12, d16, d23
	fmacd	d13, d17, d23
	fmacd	d14, d18, d23
	fmacd	d15, d19, d23

	add		r5, r5, #32
	add		r6, r6, #32

	sub		r4, r4, #1
	cmp		r4, #0
	bgt		3b

2: // return

	
#if MACRO_LEVEL>=2
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_kernel_dgemm_add_nt_4x4_lib4, .-inner_kernel_dgemm_add_nt_4x4_lib4
#endif
#endif





// subroutine
//
// input arguments:
// r4   <- k
// r5   <- A
// r6   <- B
// r7   <- 4*sdb*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_ADD_NN_4X4_LIB4
#else
//	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_dgemm_add_nn_4x4_lib4, %function
inner_kernel_dgemm_add_nn_4x4_lib4:
#elif defined(OS_MAC)
_inner_kernel_dgemm_add_nn_4x4_lib4:
#endif
#endif

	// early return
	cmp		r4, #0
	ble		2f // return

	// prefetch
	pld		[r5, #0]
	pld		[r6, #0]
#if defined(TARGET_ARMV7A_ARM_CORTEX_A9)
	pld		[r5, #32]
	pld		[r6, #32]
#endif
	pld		[r6, #64]
#if defined(TARGET_ARMV7A_ARM_CORTEX_A9)
	pld		[r6, #96]
#endif

	// preload A even
	fldd	d16, [r5, #0]
	fldd	d17, [r5, #8]
	fldd	d18, [r5, #16]
	fldd	d19, [r5, #24]

	// preload B even
	fldd	d20, [r6, #0]
	fldd	d21, [r6, #32]
	fldd	d22, [r6, #64]
	fldd	d23, [r6, #96]

	// preload A odd
	fldd	d24, [r5, #32]
	fldd	d25, [r5, #40]
	fldd	d26, [r5, #48]
	fldd	d27, [r5, #56]

	// preload B odd
	fldd	d28, [r6, #8]
	fldd	d29, [r6, #40]
	fldd	d30, [r6, #72]
	fldd	d31, [r6, #104]

	// prefetch
	pld		[r5, #64]
#if defined(TARGET_ARMV7A_ARM_CORTEX_A9)
	pld		[r5, #96]
#endif

	// B next
	add		r9, r7, r6

	cmp		r4, #4
	ble		0f // consider clean up loop

	// main loop
1:
	
	// unroll 0
	fmacd	d0, d16, d20
	pld		[r5, #128] // prefetch
	fmacd	d1, d17, d20
	pld		[r9, #0]
	fmacd	d2, d18, d20
#if defined(TARGET_ARMV7A_ARM_CORTEX_A9)
	pld		[r9, #32]
#else // cortex a7 a15
	pld		[r9, #64]
#endif
	fmacd	d3, d19, d20
	fldd	d20, [r6, #16] // B

	fmacd	d4, d16, d21
#if defined(TARGET_ARMV7A_ARM_CORTEX_A9)
	pld		[r9, #64]
#endif
	fmacd	d5, d17, d21
#if defined(TARGET_ARMV7A_ARM_CORTEX_A9)
	pld		[r9, #96]
#endif
	fmacd	d6, d18, d21
	fmacd	d7, d19, d21
	fldd	d21, [r6, #48] // B

	fmacd	d8, d16, d22
	fmacd	d9, d17, d22
	fmacd	d10, d18, d22
	fmacd	d11, d19, d22
	fldd	d22, [r6, #80] // B

	fmacd	d12, d16, d23
	fldd	d16, [r5, #64] // A
	fmacd	d13, d17, d23
	fldd	d17, [r5, #72] // A
	fmacd	d14, d18, d23
	fldd	d18, [r5, #80] // A
	fmacd	d15, d19, d23
	fldd	d19, [r5, #88] // A
	fldd	d23, [r6, #112] // B

	// unroll 1
	fmacd	d0, d24, d28
	fmacd	d1, d25, d28
	fmacd	d2, d26, d28
	fmacd	d3, d27, d28
	fldd	d28, [r6, #24] // B

	fmacd	d4, d24, d29
	fmacd	d5, d25, d29
	fmacd	d6, d26, d29
	fmacd	d7, d27, d29
	fldd	d29, [r6, #56] // B

	fmacd	d8, d24, d30
	fmacd	d9, d25, d30
	fmacd	d10, d26, d30
	fmacd	d11, d27, d30
	fldd	d30, [r6, #88] // B

	fmacd	d12, d24, d31
	fldd	d24, [r5, #96] // A
	fmacd	d13, d25, d31
	fldd	d25, [r5, #104] // A
	fmacd	d14, d26, d31
	fldd	d26, [r5, #112] // A
	fmacd	d15, d27, d31
	fldd	d27, [r5, #120] // A
	fldd	d31, [r6, #120] // B

	// unroll 2
	fmacd	d0, d16, d20
	pld		[r5, #192] // prefetch
	fmacd	d1, d17, d20
	mov		r6, r9
	fmacd	d2, d18, d20
	fmacd	d3, d19, d20
	fldd	d20, [r6, #0] // B

	fmacd	d4, d16, d21
	add		r5, r5, #128
	fmacd	d5, d17, d21
	fmacd	d6, d18, d21
	fmacd	d7, d19, d21
	fldd	d21, [r6, #32] // B

	fmacd	d8, d16, d22
	add		r9, r9, r7
	fmacd	d9, d17, d22
	fmacd	d10, d18, d22
	fmacd	d11, d19, d22
	fldd	d22, [r6, #64] // B

	fmacd	d12, d16, d23
	fldd	d16, [r5, #0] // A
	fmacd	d13, d17, d23
	fldd	d17, [r5, #8] // A
	fmacd	d14, d18, d23
	fldd	d18, [r5, #16] // A
	fmacd	d15, d19, d23
	fldd	d19, [r5, #24] // A
	fldd	d23, [r6, #96] // B

	// unroll 3
	fmacd	d0, d24, d28
	fmacd	d1, d25, d28
	fmacd	d2, d26, d28
	fmacd	d3, d27, d28
	fldd	d28, [r6, #8] // B

	fmacd	d4, d24, d29
	sub		r4, r4, #4
	fmacd	d5, d25, d29
	fmacd	d6, d26, d29
	fmacd	d7, d27, d29
	fldd	d29, [r6, #40] // B

	fmacd	d8, d24, d30
	fmacd	d9, d25, d30
	fmacd	d10, d26, d30
	fmacd	d11, d27, d30
	fldd	d30, [r6, #72] // B

	fmacd	d12, d24, d31
	fldd	d24, [r5, #32] // A
	fmacd	d13, d25, d31
	fldd	d25, [r5, #40] // A
	fmacd	d14, d26, d31
	fldd	d26, [r5, #48] // A
	fmacd	d15, d27, d31
	fldd	d27, [r5, #56] // A
	fldd	d31, [r6, #104] // B

	cmp		r4, #4
	bgt		1b

0:

	cmp		r4, #3
	ble		4f

	// unroll 0
	fmacd	d0, d16, d20
	fmacd	d1, d17, d20
	fmacd	d2, d18, d20
	fmacd	d3, d19, d20
	fldd	d20, [r6, #16] // B

	fmacd	d4, d16, d21
	fmacd	d5, d17, d21
	fmacd	d6, d18, d21
	fmacd	d7, d19, d21
	fldd	d21, [r6, #48] // B

	fmacd	d8, d16, d22
	fmacd	d9, d17, d22
	fmacd	d10, d18, d22
	fmacd	d11, d19, d22
	fldd	d22, [r6, #80] // B

	fmacd	d12, d16, d23
	fldd	d16, [r5, #64] // A
	fmacd	d13, d17, d23
	fldd	d17, [r5, #72] // A
	fmacd	d14, d18, d23
	fldd	d18, [r5, #80] // A
	fmacd	d15, d19, d23
	fldd	d19, [r5, #88] // A
	fldd	d23, [r6, #112] // B

	// unroll 1
	fmacd	d0, d24, d28
	fmacd	d1, d25, d28
	fmacd	d2, d26, d28
	fmacd	d3, d27, d28
	fldd	d28, [r6, #24] // B

	fmacd	d4, d24, d29
	fmacd	d5, d25, d29
	fmacd	d6, d26, d29
	fmacd	d7, d27, d29
	fldd	d29, [r6, #56] // B

	fmacd	d8, d24, d30
	fmacd	d9, d25, d30
	fmacd	d10, d26, d30
	fmacd	d11, d27, d30
	fldd	d30, [r6, #88] // B

	fmacd	d12, d24, d31
	fldd	d24, [r5, #96] // A
	fmacd	d13, d25, d31
	fldd	d25, [r5, #104] // A
	fmacd	d14, d26, d31
	fldd	d26, [r5, #112] // A
	fmacd	d15, d27, d31
	fldd	d27, [r5, #120] // A
	fldd	d31, [r6, #120] // B

	add		r5, r5, #128
	mov		r6, r9

	// unroll 2
	fmacd	d0, d16, d20
	fmacd	d1, d17, d20
	fmacd	d2, d18, d20
	fmacd	d3, d19, d20

	fmacd	d4, d16, d21
	fmacd	d5, d17, d21
	fmacd	d6, d18, d21
	fmacd	d7, d19, d21

	fmacd	d8, d16, d22
	fmacd	d9, d17, d22
	fmacd	d10, d18, d22
	fmacd	d11, d19, d22

	fmacd	d12, d16, d23
	fmacd	d13, d17, d23
	fmacd	d14, d18, d23
	fmacd	d15, d19, d23

	// unroll 3
	fmacd	d0, d24, d28
	fmacd	d1, d25, d28
	fmacd	d2, d26, d28
	fmacd	d3, d27, d28

	fmacd	d4, d24, d29
	fmacd	d5, d25, d29
	fmacd	d6, d26, d29
	fmacd	d7, d27, d29

	fmacd	d8, d24, d30
	fmacd	d9, d25, d30
	fmacd	d10, d26, d30
	fmacd	d11, d27, d30

	fmacd	d12, d24, d31
	fmacd	d13, d25, d31
	fmacd	d14, d26, d31
	fmacd	d15, d27, d31

	sub		r4, r4, #4

	b		2f // return

4: // consider clean1-up loop

	cmp		r4, #0
	ble		2f // return

3: // clean1-up loop

	fldd	d16, [r5, #0] // A
	fldd	d17, [r5, #8] // A
	fldd	d18, [r5, #16] // A
	fldd	d19, [r5, #24] // A

	fldd	d20, [r6, #0] // B
	fmacd	d0, d16, d20
	fmacd	d1, d17, d20
	fmacd	d2, d18, d20
	fmacd	d3, d19, d20

	fldd	d21, [r6, #32] // B
	fmacd	d4, d16, d21
	fmacd	d5, d17, d21
	fmacd	d6, d18, d21
	fmacd	d7, d19, d21

	fldd	d22, [r6, #64] // B
	fmacd	d8, d16, d22
	fmacd	d9, d17, d22
	fmacd	d10, d18, d22
	fmacd	d11, d19, d22

	fldd	d23, [r6, #96] // B
	fmacd	d12, d16, d23
	fmacd	d13, d17, d23
	fmacd	d14, d18, d23
	fmacd	d15, d19, d23

	add		r5, r5, #32
	add		r6, r6, #8

	sub		r4, r4, #1
	cmp		r4, #0
	bgt		3b

2: // return
	
#if MACRO_LEVEL>=2
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_kernel_dgemm_add_nn_4x4_lib4, .-inner_kernel_dgemm_add_nn_4x4_lib4
#endif
#endif





// subroutine
//
// input arguments:
// r4   <- k
// r5   <- A
// r6   <- B
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DSYRK_L_ADD_NT_4X4_LIB4
#else
//	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_dsyrk_l_add_nt_4x4_lib4, %function
inner_kernel_dsyrk_l_add_nt_4x4_lib4:
#elif defined(OS_MAC)
_inner_kernel_dsyrk_l_add_nt_4x4_lib4:
#endif
#endif

	// early return
	cmp		r4, #0
	ble		2f // return

	// prefetch
	pld		[r5, #0]
	pld		[r6, #0]

	// preload A even
	fldd	d16, [r5, #0]
	fldd	d17, [r5, #8]
	fldd	d18, [r5, #16]
	fldd	d19, [r5, #24]

	// preload B even
	fldd	d20, [r6, #0]
	fldd	d21, [r6, #8]
	fldd	d22, [r6, #16]
	fldd	d23, [r6, #24]

	// preload A odd
	fldd	d24, [r5, #32]
	fldd	d25, [r5, #40]
	fldd	d26, [r5, #48]
	fldd	d27, [r5, #56]

	// preload B odd
	fldd	d28, [r6, #32]
	fldd	d29, [r6, #40]
	fldd	d30, [r6, #48]
	fldd	d31, [r6, #56]

	// prefetch
	pld		[r5, #64]
	pld		[r6, #64]

	cmp		r4, #4
	ble		0f // consider clean up loop

	// main loop
1:
	
	// unroll 0
	fmacd	d0, d16, d20
	pld		[r5, #128] // prefetch
	fmacd	d1, d17, d20
	pld		[r6, #128] // prefetch
	fmacd	d2, d18, d20
	fldd	d16, [r5, #64] // A
	fmacd	d3, d19, d20
	fldd	d20, [r6, #64] // B

	fmacd	d5, d17, d21
	fldd	d17, [r5, #72] // A
	fmacd	d6, d18, d21
	fmacd	d7, d19, d21
	fldd	d21, [r6, #72] // B

	fmacd	d10, d18, d22
	fldd	d18, [r5, #80] // A
	fmacd	d11, d19, d22
	fldd	d22, [r6, #80] // B

	fmacd	d15, d19, d23
	fldd	d19, [r5, #88] // A
	fldd	d23, [r6, #88] // B

	// unroll 1
	fmacd	d0, d24, d28
	fldd	d24, [r5, #96] // A
	fmacd	d1, d25, d28
	fmacd	d2, d26, d28
	fmacd	d3, d27, d28
	fldd	d28, [r6, #96] // B

	fmacd	d5, d25, d29
	fldd	d25, [r5, #104] // A
	fmacd	d6, d26, d29
	fmacd	d7, d27, d29
	fldd	d29, [r6, #104] // B

	fmacd	d10, d26, d30
	fldd	d26, [r5, #112] // A
	fmacd	d11, d27, d30
	fldd	d30, [r6, #112] // B

	fmacd	d15, d27, d31
	fldd	d27, [r5, #120] // A
	fldd	d31, [r6, #120] // B

	// unroll 2
	fmacd	d0, d16, d20
	pld		[r6, #192] // prefetch
	fmacd	d1, d17, d20
	fldd	d16, [r5, #128] // A
	fmacd	d2, d18, d20
	add		r6, r6, #128
	fmacd	d3, d19, d20
	fldd	d20, [r6, #0] // B

	pld		[r5, #192] // prefetch
	fmacd	d5, d17, d21
	fldd	d17, [r5, #136] // A
	fmacd	d6, d18, d21
	add		r5, r5, #128
	fmacd	d7, d19, d21
	fldd	d21, [r6, #8] // B

	fmacd	d10, d18, d22
	fldd	d18, [r5, #16] // A
	fmacd	d11, d19, d22
	fldd	d22, [r6, #16] // B

	fmacd	d15, d19, d23
	fldd	d19, [r5, #24] // A
	fldd	d23, [r6, #24] // B

	// unroll 3
	fmacd	d0, d24, d28
	fldd	d24, [r5, #32] // A
	fmacd	d1, d25, d28
	sub		r4, r4, #4
	fmacd	d2, d26, d28
	fmacd	d3, d27, d28
	fldd	d28, [r6, #32] // B

	fmacd	d5, d25, d29
	fldd	d25, [r5, #40] // A
	fmacd	d6, d26, d29
	fmacd	d7, d27, d29
	fldd	d29, [r6, #40] // B

	fmacd	d10, d26, d30
	fldd	d26, [r5, #48] // A
	fmacd	d11, d27, d30
	fldd	d30, [r6, #48] // B

	fmacd	d15, d27, d31
	fldd	d27, [r5, #56] // A
	fldd	d31, [r6, #56] // B

	cmp		r4, #4
	bgt		1b

0:

	cmp		r4, #3
	ble		4f

	// unroll 0
	fmacd	d0, d16, d20
	fldd	d16, [r5, #64] // A
	fmacd	d1, d17, d20
	fmacd	d2, d18, d20
	fmacd	d3, d19, d20
	fldd	d20, [r6, #64] // B

	fmacd	d5, d17, d21
	fldd	d17, [r5, #72] // A
	fmacd	d6, d18, d21
	fmacd	d7, d19, d21
	fldd	d21, [r6, #72] // B

	fmacd	d10, d18, d22
	fldd	d18, [r5, #80] // A
	fmacd	d11, d19, d22
	fldd	d22, [r6, #80] // B

	fmacd	d15, d19, d23
	fldd	d19, [r5, #88] // A
	fldd	d23, [r6, #88] // B

	// unroll 1
	fmacd	d0, d24, d28
	fldd	d24, [r5, #96] // A
	fmacd	d1, d25, d28
	fmacd	d2, d26, d28
	fmacd	d3, d27, d28
	fldd	d28, [r6, #96] // B

	fmacd	d5, d25, d29
	fldd	d25, [r5, #104] // A
	fmacd	d6, d26, d29
	fmacd	d7, d27, d29
	fldd	d29, [r6, #104] // B

	fmacd	d10, d26, d30
	fldd	d26, [r5, #112] // A
	fmacd	d11, d27, d30
	fldd	d30, [r6, #112] // B

	fmacd	d15, d27, d31
	fldd	d27, [r5, #120] // A
	fldd	d31, [r6, #120] // B

	add		r5, r5, #128
	add		r6, r6, #128

	// unroll 2
	fmacd	d0, d16, d20
	fmacd	d1, d17, d20
	fmacd	d2, d18, d20
	fmacd	d3, d19, d20

	fmacd	d5, d17, d21
	fmacd	d6, d18, d21
	fmacd	d7, d19, d21

	fmacd	d10, d18, d22
	fmacd	d11, d19, d22

	fmacd	d15, d19, d23

	// unroll 3
	fmacd	d0, d24, d28
	fmacd	d1, d25, d28
	fmacd	d2, d26, d28
	fmacd	d3, d27, d28

	fmacd	d5, d25, d29
	fmacd	d6, d26, d29
	fmacd	d7, d27, d29

	fmacd	d10, d26, d30
	fmacd	d11, d27, d30

	fmacd	d15, d27, d31

	sub		r4, r4, #4

	b		2f // return

4: // consider clean1-up loop

	cmp		r4, #0
	ble		2f // return

3: // clean1-up loop

	fldd	d16, [r5, #0] // A
	fldd	d17, [r5, #8] // A
	fldd	d18, [r5, #16] // A
	fldd	d19, [r5, #24] // A

	fldd	d20, [r6, #0] // B
	fmacd	d0, d16, d20
	fmacd	d1, d17, d20
	fmacd	d2, d18, d20
	fmacd	d3, d19, d20

	fldd	d21, [r6, #8] // B
	fmacd	d5, d17, d21
	fmacd	d6, d18, d21
	fmacd	d7, d19, d21

	fldd	d22, [r6, #16] // B
	fmacd	d10, d18, d22
	fmacd	d11, d19, d22

	fldd	d23, [r6, #24] // B
	fmacd	d15, d19, d23

	add		r5, r5, #32
	add		r6, r6, #32

	sub		r4, r4, #1
	cmp		r4, #0
	bgt		3b

2: // return

	
#if MACRO_LEVEL>=2
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_kernel_dsyrk_l_add_nt_4x4_lib4, .-inner_kernel_dsyrk_l_add_nt_4x4_lib4
#endif
#endif





// subroutine
//
// input arguments:
// r4   <- k
// r5   <- A
// r6   <- B
// r7   <- bs*sdb*sizeof(double)
// r8   <- offsetB

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DGEMM_ADD_NN_4X4_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_dgemm_add_nn_4x4_lib4, %function
inner_edge_dgemm_add_nn_4x4_lib4:
#elif defined(OS_MAC)
_inner_edge_dgemm_add_nn_4x4_lib4:
#endif
#endif

	cmp		r8, #0
	ble		2f // return

	cmp		r4, #0
	ble		2f // return

	rsb		r9, r8, #4 // 4-offsetB
	cmp		r9, r4
//	ble		0f
//	mov		r9, r4 // kend=min(k,4-offsetB(
//0:
	movgt	r9, r4 // kend=min(k,4-offsetB(
	
//	lsl		r10, r8, #3 // offsetB*sizeof(double)
	add		r6, r6, r8, LSL #3 // B + offsetB*sizeof(double)

1:
	fldd	d16, [r5, #0] // A
	fldd	d17, [r5, #8] // A
	fldd	d18, [r5, #16] // A
	fldd	d19, [r5, #24] // A

	fldd	d20, [r6, #0] // B
	fmacd	d0, d16, d20
	fmacd	d1, d17, d20
	fmacd	d2, d18, d20
	fmacd	d3, d19, d20

	fldd	d21, [r6, #32] // B
	fmacd	d4, d16, d21
	fmacd	d5, d17, d21
	fmacd	d6, d18, d21
	fmacd	d7, d19, d21

	fldd	d22, [r6, #64] // B
	fmacd	d8, d16, d22
	fmacd	d9, d17, d22
	fmacd	d10, d18, d22
	fmacd	d11, d19, d22

	fldd	d23, [r6, #96] // B
	fmacd	d12, d16, d23
	fmacd	d13, d17, d23
	fmacd	d14, d18, d23
	fmacd	d15, d19, d23

	sub		r4, r4, #1
	sub		r9, r9, #1
	add		r5, r5, #32
	add		r6, r6, #8

	cmp		r9, #0
	bgt		1b

	cmp		r4, #0
	ble		2f // return

	add		r6, r6, r7
	sub		r6, r6, #32

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_edge_dgemm_add_nn_4x4_lib4, .-inner_edge_dgemm_add_nn_4x4_lib4
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
// r4   <- E
// r5   <- inv_diag_E
//
// output arguments:
// r4   <- E
// r5   <- inv_diag_E

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLT_INV_4X4_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_dtrsm_rlt_inv_4x4_lib4, %function
inner_edge_dtrsm_rlt_inv_4x4_lib4:
#elif defined(OS_MAC)
inner_edge_dtrsm_rlt_inv_4x4_lib4:
#endif
#endif
	
	// first column
	fldd		d16, [r5, #0] // E_inv[0]
	fmuld		d0, d0, d16
	fmuld		d1, d1, d16
	fmuld		d2, d2, d16
	fmuld		d3, d3, d16

	// second column
	fldd		d16, [r4, #8] // E[1+4*0]
	fnmacd		d4, d0, d16
	fnmacd		d5, d1, d16
	fnmacd		d6, d2, d16
	fnmacd		d7, d3, d16
	fldd		d16, [r5, #8] // E_inv[1]
	fmuld		d4, d4, d16
	fmuld		d5, d5, d16
	fmuld		d6, d6, d16
	fmuld		d7, d7, d16

	// third column
	fldd		d16, [r4, #16] // E[2+4*0]
	fnmacd		d8, d0, d16
	fnmacd		d9, d1, d16
	fnmacd		d10, d2, d16
	fnmacd		d11, d3, d16
	fldd		d16, [r4, #48] // E[2+4*1]
	fnmacd		d8, d4, d16
	fnmacd		d9, d5, d16
	fnmacd		d10, d6, d16
	fnmacd		d11, d7, d16
	fldd		d16, [r5, #16] // E_inv[2]
	fmuld		d8, d8, d16
	fmuld		d9, d9, d16
	fmuld		d10, d10, d16
	fmuld		d11, d11, d16

	// fourth column
	fldd		d16, [r4, #24] // E[3+4*0]
	fnmacd		d12, d0, d16
	fnmacd		d13, d1, d16
	fnmacd		d14, d2, d16
	fnmacd		d15, d3, d16
	fldd		d16, [r4, #56] // E[3+4*1]
	fnmacd		d12, d4, d16
	fnmacd		d13, d5, d16
	fnmacd		d14, d6, d16
	fnmacd		d15, d7, d16
	fldd		d16, [r4, #88] // E[3+4*2]
	fnmacd		d12, d8, d16
	fnmacd		d13, d9, d16
	fnmacd		d14, d10, d16
	fnmacd		d15, d11, d16
	fldd		d16, [r5, #24] // E_inv[3]
	fmuld		d12, d12, d16
	fmuld		d13, d13, d16
	fmuld		d14, d14, d16
	fmuld		d15, d15, d16
	
#if MACRO_LEVEL>=1
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_edge_dtrsm_rlt_inv_4x4_lib4, .-inner_edge_dtrsm_rlt_inv_4x4_lib4
#endif
#endif





// subroutine
//
// cholesky factorization 
//
// input arguments:
// r4   <- inv_diag_D
//
// output arguments:
// r4   <- inv_diag_D

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DPOTRF_4X4_LIB4 lc_zero
#else
	.align 3
99: // 0
	.word 0
	.word 0

	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_dpotrf_4x4_lib4, %function
inner_edge_dpotrf_4x4_lib4:
#elif defined(OS_MAC)
_inner_edge_dpotrf_4x4_lib4:
#endif
#endif
	
	fconstd		d16, #112 // 1.0
#if MACRO_LEVEL>=1
	fldd		d17, \lc_zero // 0.0
#else
	fldd		d17, 99b // 0.0
#endif

	// first column
	fcmped		d0, d17
	fmstat
	ble			1f
	fsqrtd		d0, d0
	fdivd		d18, d16, d0
	fstd		d18, [r4, #0]
2:
	fmuld		d1, d1, d18
	fmuld		d2, d2, d18
	fmuld		d3, d3, d18

	// second column
	fnmacd		d5, d1, d1
	fnmacd		d6, d1, d2
	fnmacd		d7, d1, d3
	fcmped		d5, d17
	fmstat
	ble			3f
	fsqrtd		d5, d5
	fdivd		d18, d16, d5
	fstd		d18, [r4, #8]
4:
	fmuld		d6, d6, d18
	fmuld		d7, d7, d18

	// third column
	fnmacd		d10, d2, d2
	fnmacd		d11, d2, d3
	fnmacd		d10, d6, d6
	fnmacd		d11, d6, d7
	fcmped		d10, d17
	fmstat
	ble			5f
	fsqrtd		d10, d10
	fdivd		d18, d16, d10
	fstd		d18, [r4, #16]
6:
	fmuld		d11, d11, d18

	// fourth column
	fnmacd		d15, d3, d3
	fnmacd		d15, d7, d7
	fnmacd		d15, d11, d11
	fcmped		d15, d17
	fmstat
	ble			7f
	fsqrtd		d15, d15
	fdivd		d18, d16, d15
	fstd		d18, [r4, #24]

	b			0f

1:
#if MACRO_LEVEL>=1
	fldd		d0, \lc_zero // 0.0
#else
	fldd		d0, 99b // 0.0
#endif
	b			2b

3:
#if MACRO_LEVEL>=1
	fldd		d5, \lc_zero // 0.0
#else
	fldd		d5, 99b // 0.0
#endif
	b			4b

5:
#if MACRO_LEVEL>=1
	fldd		d10, \lc_zero // 0.0
#else
	fldd		d10, 99b // 0.0
#endif
	b			6b

7:
#if MACRO_LEVEL>=1
	fldd		d15, \lc_zero // 0.0
#else
	fldd		d15, 99b // 0.0
#endif

0:
	
#if MACRO_LEVEL>=1
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_edge_dpotrf_4x4_lib4, .-inner_edge_dpotrf_4x4_lib4
#endif
#endif



// subroutine
//
// input arguments:
// r4   <- alpha
// r5   <- beta
// r6   <- C
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_SCALE_AB_4X4_LIB4 lc_zero
#else
	.align 3
99: // { 0 }
	.word 0
	.word 0
//	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_ab_4x4_lib4, %function
inner_scale_ab_4x4_lib4:
#elif defined(OS_MAC)
_inner_scale_ab_4x4_lib4:
#endif
#endif

	fldd	d16, [r4, #0] // alpha

	fmuld	d0, d0, d16
	fldd	d18, [r5, #0] // beta
	fmuld	d1, d1, d16
#if MACRO_LEVEL>=2
	fldd		d17, \lc_zero // 0.0
#else
	fldd		d17, 99b // 0.0
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

	fldd	d17, [r6, #32] // C
	fmacd	d4, d18, d17
	fldd	d17, [r6, #40] // C
	fmacd	d5, d18, d17
	fldd	d17, [r6, #48] // C
	fmacd	d6, d18, d17
	fldd	d17, [r6, #56] // C
	fmacd	d7, d18, d17

	fldd	d17, [r6, #64] // C
	fmacd	d8, d18, d17
	fldd	d17, [r6, #72] // C
	fmacd	d9, d18, d17
	fldd	d17, [r6, #80] // C
	fmacd	d10, d18, d17
	fldd	d17, [r6, #88] // C
	fmacd	d11, d18, d17

	fldd	d17, [r6, #96] // C
	fmacd	d12, d18, d17
	fldd	d17, [r6, #104] // C
	fmacd	d13, d18, d17
	fldd	d17, [r6, #112] // C
	fmacd	d14, d18, d17
	fldd	d17, [r6, #120] // C
	fmacd	d15, d18, d17

0:

#if MACRO_LEVEL>=2
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_scale_ab_4x4_lib4, .-inner_scale_ab_4x4_lib4
#endif
#endif





// subroutine
//
// input arguments:
// r4   <- beta
// r5   <- C
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_SCALE_M1B_4X4_LIB4 lc_zero
#else
	.align 3
99: // { 0 }
	.word 0
	.word 0
//	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_m1b_4x4_lib4, %function
inner_scale_m1b_4x4_lib4:
#elif defined(OS_MAC)
_inner_scale_m1b_4x4_lib4:
#endif
#endif

	fldd	d18, [r4, #0] // beta
#if MACRO_LEVEL>=2
	fldd	d17, \lc_zero // 0.0
#else
	fldd	d17, 99b // 0.0
#endif
	fcmped	d18, d17
	fmstat
	beq		0f // end

	fldd	d17, [r5, #0] // C
	fmscd	d0, d17, d18
	fldd	d17, [r5, #8] // C
	fmscd	d1, d17, d18
	fldd	d17, [r5, #16] // C
	fmscd	d2, d17, d18
	fldd	d17, [r5, #24] // C
	fmscd	d3, d17, d18

	fldd	d17, [r5, #32] // C
	fmscd	d4, d17, d18
	fldd	d17, [r5, #40] // C
	fmscd	d5, d17, d18
	fldd	d17, [r5, #48] // C
	fmscd	d6, d17, d18
	fldd	d17, [r5, #56] // C
	fmscd	d7, d17, d18

	fldd	d17, [r5, #64] // C
	fmscd	d8, d17, d18
	fldd	d17, [r5, #72] // C
	fmscd	d9, d17, d18
	fldd	d17, [r5, #80] // C
	fmscd	d10, d17, d18
	fldd	d17, [r5, #88] // C
	fmscd	d11, d17, d18

	fldd	d17, [r5, #96] // C
	fmscd	d12, d17, d18
	fldd	d17, [r5, #104] // C
	fmscd	d13, d17, d18
	fldd	d17, [r5, #112] // C
	fmscd	d14, d17, d18
	fldd	d17, [r5, #120] // C
	fmscd	d15, d17, d18

0:

#if MACRO_LEVEL>=2
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_scale_m1b_4x4_lib4, .-inner_scale_m1b_4x4_lib4
#endif
#endif





// subroutine
//
// input arguments:
// r4   <- C
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_SCALE_M11_4X4_LIB4
#else
//	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_m11_4x4_lib4, %function
inner_scale_m11_4x4_lib4:
#elif defined(OS_MAC)
_inner_scale_m11_4x4_lib4:
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

	fldd	d17, [r4, #32] // C
	fsubd	d4, d17, d4
	fldd	d17, [r4, #40] // C
	fsubd	d5, d17, d5
	fldd	d17, [r4, #48] // C
	fsubd	d6, d17, d6
	fldd	d17, [r4, #56] // C
	fsubd	d7, d17, d7

	fldd	d17, [r4, #64] // C
	fsubd	d8, d17, d8
	fldd	d17, [r4, #72] // C
	fsubd	d9, d17, d9
	fldd	d17, [r4, #80] // C
	fsubd	d10, d17, d10
	fldd	d17, [r4, #88] // C
	fsubd	d11, d17, d11

	fldd	d17, [r4, #96] // C
	fsubd	d12, d17, d12
	fldd	d17, [r4, #104] // C
	fsubd	d13, d17, d13
	fldd	d17, [r4, #112] // C
	fsubd	d14, d17, d14
	fldd	d17, [r4, #120] // C
	fsubd	d15, d17, d15


#if MACRO_LEVEL>=2
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_scale_m11_4x4_lib4, .-inner_scale_m11_4x4_lib4
#endif
#endif





// subroutine
//
// input arguments:
// r4   <- D
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_STORE_4X4_LIB4
#else
//	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_4x4_lib4, %function
inner_store_4x4_lib4:
#elif defined(OS_MAC)
_inner_store_4x4_lib4:
#endif
#endif

	fstd	d0, [r4, #0]
	fstd	d1, [r4, #8]
	fstd	d2, [r4, #16]
	fstd	d3, [r4, #24]

	fstd	d4, [r4, #32]
	fstd	d5, [r4, #40]
	fstd	d6, [r4, #48]
	fstd	d7, [r4, #56]

	fstd	d8, [r4, #64]
	fstd	d9, [r4, #72]
	fstd	d10, [r4, #80]
	fstd	d11, [r4, #88]

	fstd	d12, [r4, #96]
	fstd	d13, [r4, #104]
	fstd	d14, [r4, #112]
	fstd	d15, [r4, #120]

#if MACRO_LEVEL>=2
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_store_4x4_lib4, .-inner_store_4x4_lib4
#endif
#endif





// subroutine
//
// input arguments:
// r4   <- D
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_STORE_L_4X4_LIB4
#else
//	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_l_4x4_lib4, %function
inner_store_l_4x4_lib4:
#elif defined(OS_MAC)
_inner_store_l_4x4_lib4:
#endif
#endif

	fstd	d0, [r4, #0]
	fstd	d1, [r4, #8]
	fstd	d2, [r4, #16]
	fstd	d3, [r4, #24]

//	fstd	d4, [r4, #32]
	fstd	d5, [r4, #40]
	fstd	d6, [r4, #48]
	fstd	d7, [r4, #56]

//	fstd	d8, [r4, #64]
//	fstd	d9, [r4, #72]
	fstd	d10, [r4, #80]
	fstd	d11, [r4, #88]

//	fstd	d12, [r4, #96]
//	fstd	d13, [r4, #104]
//	fstd	d14, [r4, #112]
	fstd	d15, [r4, #120]

#if MACRO_LEVEL>=2
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_store_l_4x4_lib4, .-inner_store_l_4x4_lib4
#endif
#endif





// zero double word
	.align 3
99: // 0
	.word 0
	.word 0

//                               r0        r1             r2         r3         sp+0          sp+4       sp+8
// void kernel_dgemm_nt_4x4_lib4(int kmax, double *alpha, double *A, double *B, double *beta, double *C, double *D)

//	.p2align 4,,15
#if defined(OS_LINUX)
	.global	kernel_dgemm_nt_4x4_lib4
	.type	kernel_dgemm_nt_4x4_lib4, %function
kernel_dgemm_nt_4x4_lib4:
#elif defined(OS_MAC)
	.global	_kernel_dgemm_nt_4x4_lib4
_kernel_dgemm_nt_4x4_lib4:
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

#if MACRO_LEVEL>=2
	INNER_SCALE_AB_4X4_LIB4 99f
#else
#if defined(OS_LINUX)
	bl inner_scale_ab_4x4_lib4
#elif defined(OS_MAC)
	bl _inner_scale_ab_4x4_lib4
#endif
#endif



	// store n
	ldr		r4, [fp, #8] // D

#if MACRO_LEVEL>=2
	INNER_STORE_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl inner_store_4x4_lib4
#elif defined(OS_MAC)
	bl _inner_store_4x4_lib4
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
	.size	kernel_dgemm_nt_4x4_lib4, .-kernel_dgemm_nt_4x4_lib4
#endif





	.align 3
99: // 0
	.word 0
	.word 0





//                               r0        r1             r2         r3           sp+0       sp+4     sp+8          sp+12      sp+16
// void kernel_dgemm_nn_4x4_lib4(int kmax, double *alpha, double *A, int offsetB, double *B, int sdb, double *beta, double *C, double *D)

//	.p2align 4,,15
#if defined(OS_LINUX)
	.global	kernel_dgemm_nn_4x4_lib4
	.type	kernel_dgemm_nn_4x4_lib4, %function
kernel_dgemm_nn_4x4_lib4:
#elif defined(OS_MAC)
	.global	_kernel_dgemm_nn_4x4_lib4
_kernel_dgemm_nn_4x4_lib4:
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
	ldr		r6, [fp, #0] // B
	ldr		r7, [fp, #4] // sdb
	lsl		r7, r7, #5 // 4*sizeof(double)*sdb
	mov		r8, r3 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_ADD_NN_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl	inner_edge_dgemm_add_nn_4x4_lib4
#elif defined(OS_MAC)
	bl	_inner_edge_dgemm_add_nn_4x4_lib4
#endif
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_ADD_NN_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl	inner_kernel_dgemm_add_nn_4x4_lib4
#elif defined(OS_MAC)
	bl	_inner_kernel_dgemm_add_nn_4x4_lib4
#endif
#endif



	// call inner blend for generic alpha and beta
	mov		r4, r1 // alpha
	ldr		r5, [fp, #8] // beta
	ldr		r6, [fp, #12] // C

#if MACRO_LEVEL>=2
	INNER_SCALE_AB_4X4_LIB4 99f
#else
#if defined(OS_LINUX)
	bl inner_scale_ab_4x4_lib4
#elif defined(OS_MAC)
	bl _inner_scale_ab_4x4_lib4
#endif
#endif



	// store n
	ldr		r4, [fp, #16] // D

#if MACRO_LEVEL>=2
	INNER_STORE_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl inner_store_4x4_lib4
#elif defined(OS_MAC)
	bl _inner_store_4x4_lib4
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
	.size	kernel_dgemm_nn_4x4_lib4, .-kernel_dgemm_nn_4x4_lib4
#endif





	.align 3
99: // 0
	.word 0
	.word 0





//                                 r0        r1             r2         r3         sp+0          sp+4       sp+8
// void kernel_dsyrk_nt_l_4x4_lib4(int kmax, double *alpha, double *A, double *B, double *beta, double *C, double *D);

//	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dsyrk_nt_l_4x4_lib4
	.type kernel_dsyrk_nt_l_4x4_lib4, %function
kernel_dsyrk_nt_l_4x4_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dsyrk_nt_l_4x4_lib4
_kernel_dsyrk_nt_l_4x4_lib4:
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
	mov		r5, r2 // A
	mov		r6, r3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DSYRK_L_ADD_NT_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl	inner_kernel_dsyrk_l_add_nt_4x4_lib4
#elif defined(OS_MAC)
	bl	_inner_kernel_dsyrk_l_add_nt_4x4_lib4
#endif
#endif



	// call inner blend for generic alpha and beta
	mov		r4, r1 // alpha
	ldr		r5, [fp, #0] // beta
	ldr		r6, [fp, #4] // C

#if MACRO_LEVEL>=2
	INNER_SCALE_AB_4X4_LIB4 99f
#else
#if defined(OS_LINUX)
	bl inner_scale_ab_4x4_lib4
#elif defined(OS_MAC)
	bl _inner_scale_ab_4x4_lib4
#endif
#endif



	// store l
	ldr		r4, [fp, #8] // D

#if MACRO_LEVEL>=2
	INNER_STORE_L_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl inner_store_l_4x4_lib4
#elif defined(OS_MAC)
	bl _inner_store_l_4x4_lib4
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
	.size	kernel_dsyrk_nt_l_4x4_lib4, .-kernel_dsyrk_nt_l_4x4_lib4
#endif





	.align 3
99: // 0
	.word 0
	.word 0





//                                      r0        r1         r2         r3            sp+0       sp+4       sp+8       sp+12
// void kernel_dtrsm_nt_rl_inv_4x4_lib4(int kmax, double *A, double *B, double *beta, double *C, double *D, double *E, double *inv_diag_E);

//	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dtrsm_nt_rl_inv_4x4_lib4
	.type kernel_dtrsm_nt_rl_inv_4x4_lib4, %function
kernel_dtrsm_nt_rl_inv_4x4_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dtrsm_nt_rl_inv_4x4_lib4
_kernel_dtrsm_nt_rl_inv_4x4_lib4:
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



	// call inner blend for alpha=1.0 and beta
	mov		r4, r3 // beta
	ldr		r5, [fp, #0] // C

#if MACRO_LEVEL>=2
	INNER_SCALE_M1B_4X4_LIB4 99f
#else
#if defined(OS_LINUX)
	bl inner_scale_m1b_4x4_lib4
#elif defined(OS_MAC)
	bl _inner_scale_m1b_4x4_lib4
#endif
#endif



	// solution
	ldr		r4, [fp, #8] // E
	ldr		r5, [fp, #12] // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl inner_edge_dtrsm_rlt_inv_4x4_lib4
#elif defined(OS_MAC)
	bl _inner_edge_dtrsm_rlt_inv_4x4_lib4
#endif
#endif



	// store
	ldr		r4, [fp, #4] // D

#if MACRO_LEVEL>=2
	INNER_STORE_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl inner_store_4x4_lib4
#elif defined(OS_MAC)
	bl _inner_store_4x4_lib4
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
	.size	kernel_dtrsm_nt_rl_inv_4x4_lib4, .-kernel_dtrsm_nt_rl_inv_4x4_lib4
#endif





	.align 3
99: // 0
	.word 0
	.word 0





//                                  r0        r1         r2         r3         sp+0       sp+4
// void kernel_dpotrf_nt_l_4x4_lib4(int kmax, double *A, double *B, double *C, double *D, double *inv_diag_D);

//	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dpotrf_nt_l_4x4_lib4
	.type kernel_dpotrf_nt_l_4x4_lib4, %function
kernel_dpotrf_nt_l_4x4_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dpotrf_nt_l_4x4_lib4
_kernel_dpotrf_nt_l_4x4_lib4:
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

#if MACRO_LEVEL>=2
	INNER_SCALE_M11_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl inner_scale_m11_4x4_lib4
#elif defined(OS_MAC)
	bl _inner_scale_m11_4x4_lib4
#endif
#endif



	// factorization
	ldr		r4, [fp, #4] // inv_diag_D

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
	ldr		r4, [fp, #0] // D

#if MACRO_LEVEL>=2
	INNER_STORE_L_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl inner_store_l_4x4_lib4
#elif defined(OS_MAC)
	bl _inner_store_l_4x4_lib4
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
	.size	kernel_dpotrf_nt_l_4x4_lib4, .-kernel_dpotrf_nt_l_4x4_lib4
#endif





	.align 3
99: // 0
	.word 0
	.word 0





//                                            r0      r1          r2          r3      sp+0        sp+4        sp+8       sp+12      sp+16      sp+20
// void kernel_dgemm_dtrsm_nt_rl_inv_4x4_lib4(int kp, double *Ap, double *Bp, int km, double *Am, double *Bm, double *C, double *D, double *E, double *inv_diag_E);

//	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dgemm_dtrsm_nt_rl_inv_4x4_lib4
	.type kernel_dgemm_dtrsm_nt_rl_inv_4x4_lib4, %function
kernel_dgemm_dtrsm_nt_rl_inv_4x4_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dgemm_dtrsm_nt_rl_inv_4x4_lib4
_kernel_dgemm_dtrsm_nt_rl_inv_4x4_lib4:
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



	// call inner kernel dsyrk l nt add
	mov		r4, r0 // kp
	mov		r5, r1 // Ap
	mov		r6, r2 // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_ADD_NT_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl	inner_kernel_dgemm_add_nt_4x4_lib4
#elif defined(OS_MAC)
	bl	_inner_kernel_dgemm_add_nt_4x4_lib4
#endif
#endif



	fnegd	d0, d0
	fnegd	d1, d1
	fnegd	d2, d2
	fnegd	d3, d3
	fnegd	d4, d4
	fnegd	d5, d5
	fnegd	d6, d6
	fnegd	d7, d7
	fnegd	d8, d8
	fnegd	d9, d9
	fnegd	d10, d10
	fnegd	d11, d11
	fnegd	d12, d12
	fnegd	d13, d13
	fnegd	d14, d14
	fnegd	d15, d15



	// call inner kernel dsyrk l nt sub
	mov		r4, r3 // kmax
	ldr		r5, [fp, #0] // Am
	ldr		r6, [fp, #4] // Bm

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
	ldr		r4, [fp, #8] // C

#if MACRO_LEVEL>=2
	INNER_SCALE_M11_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl inner_scale_m11_4x4_lib4
#elif defined(OS_MAC)
	bl _inner_scale_m11_4x4_lib4
#endif
#endif



	// factorization
	ldr		r4, [fp, #16] // E
	ldr		r5, [fp, #20] // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl inner_edge_dtrsm_rlt_inv_4x4_lib4
#elif defined(OS_MAC)
	bl _inner_edge_dtrsm_rlt_inv_4x4_lib4
#endif
#endif



	// store l
	ldr		r4, [fp, #12] // D

#if MACRO_LEVEL>=2
	INNER_STORE_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl inner_store_4x4_lib4
#elif defined(OS_MAC)
	bl _inner_store_4x4_lib4
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
	.size	kernel_dgemm_dtrsm_nt_rl_inv_4x4_lib4, .-kernel_dgemm_dtrsm_nt_rl_inv_4x4_lib4
#endif





	.align 3
99: // 0
	.word 0
	.word 0





//                                        r0      r1          r2          r3      sp+0        sp+4        sp+8       sp+12      sp+16
// void kernel_dsyrk_dpotrf_nt_l_4x4_lib4(int kp, double *Ap, double *Bp, int km, double *Am, double *Bm, double *C, double *D, double *inv_diag_D);

//	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dsyrk_dpotrf_nt_l_4x4_lib4
	.type kernel_dsyrk_dpotrf_nt_l_4x4_lib4, %function
kernel_dsyrk_dpotrf_nt_l_4x4_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dpotrf_nt_l_4x4_lib4
_kernel_dsyrk_dpotrf_nt_l_4x4_lib4:
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
	mov		r4, r0 // kp
	mov		r5, r1 // Ap
	mov		r6, r2 // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_DSYRK_L_ADD_NT_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl	inner_kernel_dsyrk_l_add_nt_4x4_lib4
#elif defined(OS_MAC)
	bl	_inner_kernel_dsyrk_l_add_nt_4x4_lib4
#endif
#endif



	fnegd	d0, d0
	fnegd	d1, d1
	fnegd	d2, d2
	fnegd	d3, d3
	fnegd	d4, d4
	fnegd	d5, d5
	fnegd	d6, d6
	fnegd	d7, d7
	fnegd	d8, d8
	fnegd	d9, d9
	fnegd	d10, d10
	fnegd	d11, d11
	fnegd	d12, d12
	fnegd	d13, d13
	fnegd	d14, d14
	fnegd	d15, d15



	// call inner kernel dsyrk l nt sub
	mov		r4, r3 // kmax
	ldr		r5, [fp, #0] // Am
	ldr		r6, [fp, #4] // Bm

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
	ldr		r4, [fp, #8] // C

#if MACRO_LEVEL>=2
	INNER_SCALE_M11_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl inner_scale_m11_4x4_lib4
#elif defined(OS_MAC)
	bl _inner_scale_m11_4x4_lib4
#endif
#endif



	// factorization
	ldr		r4, [fp, #16] // inv_diag_D

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
	ldr		r4, [fp, #12] // D

#if MACRO_LEVEL>=2
	INNER_STORE_L_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl inner_store_l_4x4_lib4
#elif defined(OS_MAC)
	bl _inner_store_l_4x4_lib4
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
	.size	kernel_dsyrk_dpotrf_nt_l_4x4_lib4, .-kernel_dsyrk_dpotrf_nt_l_4x4_lib4
#endif





	.align 3
99: // 0
	.word 0
	.word 0





//#if defined(BLAS_API)
#if ( defined(BLAS_API) | ( defined(LA_HIGH_PERFORMANCE) & defined(MF_COLMAJ) ) )

#include "kernel_dgemm_4x4_lib.S"

#endif






