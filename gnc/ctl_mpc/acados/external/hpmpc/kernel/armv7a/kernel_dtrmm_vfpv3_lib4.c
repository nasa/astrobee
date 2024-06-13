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



void kernel_dtrmm_nt_u_4x4_lib4(int kmax, double *A, double *B, double *C)
	{

	__builtin_prefetch( A );
	__builtin_prefetch( B );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A+4 );
	__builtin_prefetch( B+4 );
#endif

	int k_iter = kmax/4 - 1;
	int k_left = kmax%4;

	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"pld    [%2, #64]                \n\t"
		"pld    [%3, #64]                \n\t"
#if defined(TARGET_CORTEX_A9)
		"pld    [%2, #96]                \n\t"
		"pld    [%3, #96]                \n\t"
#endif
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"fldd   d16, [%2, #0]            \n\t" // prefetch A_even
		"fldd   d17, [%2, #8]            \n\t"
		"fldd   d18, [%2, #16]           \n\t"
		"fldd   d19, [%2, #24]           \n\t"
		"                                \n\t"
		"fldd   d20, [%3, #0]            \n\t" // prefetch B_even
		"                                \n\t"
		"fldd   d24, [%2, #32]           \n\t" // prefetch A_odd
		"fldd   d25, [%2, #40]           \n\t"
		"fldd   d26, [%2, #48]           \n\t"
		"fldd   d27, [%2, #56]           \n\t"
		"                                \n\t"
		"fldd   d28, [%3, #32]           \n\t" // prefetch B_odd
		"fldd   d29, [%3, #40]           \n\t"
		"                                \n\t"
		"                                \n\t"
		"pld    [%2, #128]               \n\t"
		"pld    [%3, #128]               \n\t"
		"                                \n\t"
		"                                \n\t"
		"fmuld  d0, d16, d20             \n\t"
		"fldd   d16, [%2, #64]           \n\t" // prefetch A_even
		"fmuld  d1, d17, d20             \n\t"
		"fldd   d17, [%2, #72]           \n\t"
		"fmuld  d2, d18, d20             \n\t"
		"fldd   d18, [%2, #80]           \n\t"
		"fmuld  d3, d19, d20             \n\t"
		"fldd   d20, [%3, #64]           \n\t" // prefetch B_even
		"                                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A9)
		"pld    [%2, #160]               \n\t"
		"pld    [%3, #160]               \n\t"
#endif
		"                                \n\t"
		"                                \n\t"
		"fmacd  d0, d24, d28             \n\t"
		"fldd   d19, [%2, #88]           \n\t"
		"fmacd  d1, d25, d28             \n\t"
		"fldd   d21, [%3, #72]           \n\t"
		"sub    r3, r3, #1               \n\t" // iter++
		"fmacd  d2, d26, d28             \n\t"
		"fldd   d22, [%3, #80]           \n\t"
		"fmacd  d3, d27, d28             \n\t"
		"fldd   d28, [%3, #96]           \n\t" // prefetch B_odd
		"                                \n\t"
		"fmuld  d4, d24, d29             \n\t"
		"fldd   d24, [%2, #96]           \n\t" // prefetch A_odd
		"fmuld  d5, d25, d29             \n\t"
		"fldd   d25, [%2, #104]          \n\t"
		"fmuld  d6, d26, d29             \n\t"
		"fldd   d26, [%2, #112]          \n\t"
		"fmuld  d7, d27, d29             \n\t"
		"fldd   d29, [%3, #104]          \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"pld    [%2, #192]               \n\t"
		"pld    [%3, #192]               \n\t"
		"                                \n\t"
		"                                \n\t"
		"fmacd  d0, d16, d20             \n\t"
		"fldd   d27, [%2, #120]          \n\t"
		"fmacd  d1, d17, d20             \n\t"
		"fldd   d30, [%3, #112]          \n\t"
		"fmacd  d2, d18, d20             \n\t"
		"fldd   d31, [%3, #120]          \n\t"
		"fmacd  d3, d19, d20             \n\t"
		"fldd   d20, [%3, #128]          \n\t" // prefetch B_even
		"                                \n\t"
		"fmacd  d4, d16, d21             \n\t"
		"cmp    r3, #0                   \n\t" // next iter?
		"fmacd  d5, d17, d21             \n\t"
		"fmacd  d6, d18, d21             \n\t"
		"fmacd  d7, d19, d21             \n\t"
		"fldd   d21, [%3, #136]          \n\t"
		"                                \n\t"
		"fmuld  d8, d16, d22             \n\t"
		"fldd   d16, [%2, #128]          \n\t" // prefetch A_even
		"fmuld  d9, d17, d22             \n\t"
		"fldd   d17, [%2, #136]          \n\t"
		"fmuld  d10, d18, d22            \n\t"
		"fldd   d18, [%2, #144]          \n\t"
		"fmuld  d11, d19, d22            \n\t"
		"fldd   d22, [%3, #144]          \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A9)
		"pld    [%2, #224]               \n\t"
		"pld    [%3, #224]               \n\t"
#endif
		"                                \n\t"
		"                                \n\t"
		"fmacd  d0, d24, d28             \n\t"
		"fldd   d19, [%2, #152]          \n\t"
		"add    %2, %2, #128             \n\t" // increase A
		"fmacd  d1, d25, d28             \n\t"
		"fldd   d23, [%3, #152]          \n\t"
		"fmacd  d2, d26, d28             \n\t"
		"add    %3, %3, #128             \n\t" // increase B
		"fmacd  d3, d27, d28             \n\t"
		"fldd   d28, [%3, #32]           \n\t" // prefetch B_odd
		"                                \n\t"
		"fmacd  d4, d24, d29             \n\t"
		"fmacd  d5, d25, d29             \n\t"
		"fmacd  d6, d26, d29             \n\t"
		"fmacd  d7, d27, d29             \n\t"
		"fldd   d29, [%3, #40]           \n\t"
		"                                \n\t"
		"fmacd  d8, d24, d30             \n\t"
		"fmacd  d9, d25, d30             \n\t"
		"fmacd  d10, d26, d30            \n\t"
		"fmacd  d11, d27, d30            \n\t"
		"fldd   d30, [%3, #48]           \n\t"
		"                                \n\t"
		"fmuld  d12, d24, d31            \n\t"
		"fldd   d24, [%2, #32]           \n\t" // prefetch A_odd
		"fmuld  d13, d25, d31            \n\t"
		"fldd   d25, [%2, #40]           \n\t"
		"fmuld  d14, d26, d31            \n\t"
		"fldd   d26, [%2, #48]           \n\t"
		"fmuld  d15, d27, d31            \n\t"
		"fldd   d31, [%3, #56]           \n\t"
		"fldd   d27, [%2, #56]           \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"mov    r3, %0                   \n\t" // k_iter
		"cmp    r3, #0                   \n\t"
		"ble    .DCONSIDERLEFT           \n\t"
		"                                \n\t"
		"                                \n\t"
		".DLOOPKITER:                    \n\t" // main loop
		"                                \n\t"
		"                                \n\t"
		"pld    [%2, #128]               \n\t"
		"pld    [%3, #128]               \n\t"
		"                                \n\t"
		"                                \n\t"
		"fmacd  d0, d16, d20             \n\t"
		"fmacd  d1, d17, d20             \n\t"
		"fmacd  d2, d18, d20             \n\t"
		"fmacd  d3, d19, d20             \n\t"
		"fldd   d20, [%3, #64]           \n\t" // prefetch B_even
		"                                \n\t"
		"fmacd  d4, d16, d21             \n\t"
		"fmacd  d5, d17, d21             \n\t"
		"fmacd  d6, d18, d21             \n\t"
		"fmacd  d7, d19, d21             \n\t"
		"fldd   d21, [%3, #72]           \n\t"
		"                                \n\t"
		"fmacd  d8, d16, d22             \n\t"
		"fmacd  d9, d17, d22             \n\t"
		"fmacd  d10, d18, d22            \n\t"
		"fmacd  d11, d19, d22            \n\t"
		"fldd   d22, [%3, #80]           \n\t"
		"                                \n\t"
		"fmacd  d12, d16, d23            \n\t"
		"fldd   d16, [%2, #64]           \n\t" // prefetch A_even
		"fmacd  d13, d17, d23            \n\t"
		"fldd   d17, [%2, #72]           \n\t"
		"fmacd  d14, d18, d23            \n\t"
		"fldd   d18, [%2, #80]           \n\t"
		"fmacd  d15, d19, d23            \n\t"
		"fldd   d23, [%3, #88]           \n\t"
		"                                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A9)
		"pld    [%2, #160]               \n\t"
		"pld    [%3, #160]               \n\t"
#endif
		"                                \n\t"
		"                                \n\t"
		"fmacd  d0, d24, d28             \n\t"
		"fldd   d19, [%2, #88]           \n\t"
		"fmacd  d1, d25, d28             \n\t"
		"sub    r3, r3, #1               \n\t" // iter++
		"fmacd  d2, d26, d28             \n\t"
		"fmacd  d3, d27, d28             \n\t"
		"fldd   d28, [%3, #96]           \n\t" // prefetch B_odd
		"                                \n\t"
		"fmacd  d4, d24, d29             \n\t"
		"fmacd  d5, d25, d29             \n\t"
		"fmacd  d6, d26, d29             \n\t"
		"fmacd  d7, d27, d29             \n\t"
		"fldd   d29, [%3, #104]          \n\t"
		"                                \n\t"
		"fmacd  d8, d24, d30             \n\t"
		"fmacd  d9, d25, d30             \n\t"
		"fmacd  d10, d26, d30            \n\t"
		"fmacd  d11, d27, d30            \n\t"
		"fldd   d30, [%3, #112]          \n\t"
		"                                \n\t"
		"fmacd  d12, d24, d31            \n\t"
		"fldd   d24, [%2, #96]           \n\t" // prefetch A_odd
		"fmacd  d13, d25, d31            \n\t"
		"fldd   d25, [%2, #104]          \n\t"
		"fmacd  d14, d26, d31            \n\t"
		"fldd   d26, [%2, #112]          \n\t"
		"fmacd  d15, d27, d31            \n\t"
		"fldd   d31, [%3, #120]          \n\t"
		"                                \n\t"
		"                                \n\t"
		"pld    [%2, #192]               \n\t"
		"pld    [%3, #192]               \n\t"
		"                                \n\t"
		"                                \n\t"
		"fmacd  d0, d16, d20             \n\t"
		"fldd   d27, [%2, #120]          \n\t"
		"fmacd  d1, d17, d20             \n\t"
		"cmp    r3, #0                   \n\t" // next iter?
		"fmacd  d2, d18, d20             \n\t"
		"fmacd  d3, d19, d20             \n\t"
		"fldd   d20, [%3, #128]          \n\t" // prefetch B_even
		"                                \n\t"
		"fmacd  d4, d16, d21             \n\t"
		"fmacd  d5, d17, d21             \n\t"
		"fmacd  d6, d18, d21             \n\t"
		"fmacd  d7, d19, d21             \n\t"
		"fldd   d21, [%3, #136]          \n\t"
		"                                \n\t"
		"fmacd  d8, d16, d22             \n\t"
		"fmacd  d9, d17, d22             \n\t"
		"fmacd  d10, d18, d22            \n\t"
		"fmacd  d11, d19, d22            \n\t"
		"fldd   d22, [%3, #144]          \n\t"
		"                                \n\t"
		"fmacd  d12, d16, d23            \n\t"
		"fldd   d16, [%2, #128]          \n\t" // prefetch A_even
		"fmacd  d13, d17, d23            \n\t"
		"fldd   d17, [%2, #136]          \n\t"
		"fmacd  d14, d18, d23            \n\t"
		"fldd   d18, [%2, #144]          \n\t"
		"fmacd  d15, d19, d23            \n\t"
		"fldd   d19, [%2, #152]          \n\t"
		"                                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A9)
		"pld    [%2, #224]               \n\t"
		"pld    [%3, #224]               \n\t"
#endif
		"                                \n\t"
		"                                \n\t"
		"fmacd  d0, d24, d28             \n\t"
		"add    %2, %2, #128             \n\t" // increase A
		"fmacd  d1, d25, d28             \n\t"
		"fldd   d23, [%3, #152]          \n\t"
		"fmacd  d2, d26, d28             \n\t"
		"add    %3, %3, #128             \n\t" // increase B
		"fmacd  d3, d27, d28             \n\t"
		"fldd   d28, [%3, #32]           \n\t" // prefetch B_odd
		"                                \n\t"
		"fmacd  d4, d24, d29             \n\t"
		"fmacd  d5, d25, d29             \n\t"
		"fmacd  d6, d26, d29             \n\t"
		"fmacd  d7, d27, d29             \n\t"
		"fldd   d29, [%3, #40]           \n\t"
		"                                \n\t"
		"fmacd  d8, d24, d30             \n\t"
		"fmacd  d9, d25, d30             \n\t"
		"fmacd  d10, d26, d30            \n\t"
		"fmacd  d11, d27, d30            \n\t"
		"fldd   d30, [%3, #48]           \n\t"
		"                                \n\t"
		"fmacd  d12, d24, d31            \n\t"
		"fldd   d24, [%2, #32]           \n\t" // prefetch A_odd
		"fmacd  d13, d25, d31            \n\t"
		"fldd   d25, [%2, #40]           \n\t"
		"fmacd  d14, d26, d31            \n\t"
		"fldd   d26, [%2, #48]           \n\t"
		"fmacd  d15, d27, d31            \n\t"
		"fldd   d31, [%3, #56]           \n\t"
		"fldd   d27, [%2, #56]           \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"bgt    .DLOOPKITER              \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERLEFT:                 \n\t" // consider left
		"                                \n\t"
		"pld   [%4, #0]                 \n\t" // prefetch C
		"pld   [%4, #32]                \n\t"
		"pld   [%4, #64]                \n\t"
		"pld   [%4, #96]                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"mov    r4, %1                   \n\t" // k_left
		"cmp    r4, #0                   \n\t"
		"ble    .DPOSTACCUM              \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DLOOPKLEFT:                    \n\t" // clean up loop
		"                                \n\t"
		"sub    r4, r4, #1               \n\t"
		"                                \n\t"
		"fmacd  d0, d16, d20             \n\t"
		"fmacd  d1, d17, d20             \n\t"
		"fmacd  d2, d18, d20             \n\t"
		"fmacd  d3, d19, d20             \n\t"
		"fldd   d20, [%3, #32]           \n\t" // prefetch B_even
		"                                \n\t"
		"fmacd  d4, d16, d21             \n\t"
		"fmacd  d5, d17, d21             \n\t"
		"fmacd  d6, d18, d21             \n\t"
		"fmacd  d7, d19, d21             \n\t"
		"fldd   d21, [%3, #40]           \n\t"
		"                                \n\t"
		"fmacd  d8, d16, d22             \n\t"
		"fmacd  d9, d17, d22             \n\t"
		"fmacd  d10, d18, d22            \n\t"
		"fmacd  d11, d19, d22            \n\t"
		"fldd   d22, [%3, #48]           \n\t"
		"                                \n\t"
		"cmp    r4, #0                   \n\t"
		"                                \n\t"
		"fmacd  d12, d16, d23            \n\t"
		"fldd   d16, [%2, #32]           \n\t" // prefetch A_even
		"fmacd  d13, d17, d23            \n\t"
		"fldd   d17, [%2, #40]           \n\t"
		"fmacd  d14, d18, d23            \n\t"
		"fldd   d18, [%2, #48]           \n\t"
		"fmacd  d15, d19, d23            \n\t"
		"fldd   d19, [%2, #56]           \n\t"
		"add    %2, %2, #32              \n\t"
		"fldd   d23, [%3, #56]           \n\t"
		"add    %3, %3, #32              \n\t"
		"                                \n\t"
		"                                \n\t"
		"bgt    .DLOOPKLEFT              \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DPOSTACCUM:                    \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"fstd   d0, [%4, #0]             \n\t" // store result
		"fstd   d1, [%4, #8]             \n\t"
		"fstd   d2, [%4, #16]            \n\t"
		"fstd   d3, [%4, #24]            \n\t"
		"                                \n\t"
		"fstd   d4, [%4, #32]            \n\t"
		"fstd   d5, [%4, #40]            \n\t"
		"fstd   d6, [%4, #48]            \n\t"
		"fstd   d7, [%4, #56]            \n\t"
		"                                \n\t"
		"fstd   d8, [%4, #64]            \n\t"
		"fstd   d9, [%4, #72]            \n\t"
		"fstd   d10, [%4, #80]           \n\t"
		"fstd   d11, [%4, #88]           \n\t"
		"                                \n\t"
		"fstd   d12, [%4, #96]           \n\t"
		"fstd   d13, [%4, #104]          \n\t"
		"fstd   d14, [%4, #112]          \n\t"
		"fstd   d15, [%4, #120]          \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		: // output operands (none)
		: // input operands
		  "r" (k_iter),		// %0
		  "r" (k_left),		// %1
		  "r" (A),			// %2
		  "r" (B),			// %3
		  "r" (C)			// %4
		: // register clobber list
		  "r3", "r4", "r5",
		  "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",
		  "d8", "d9", "d10", "d11", "d12", "d13", "d14", "d15",
		  "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23",
		  "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31",
		  "memory"
	);

	}



void corner_dtrmm_nt_u_4x3_lib4(double *A, double *B, double *C)
	{

	const int bs = 4;
	const int lda = bs;
	const int ldc = 4;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0, c_02=0,
		c_10=0, c_11=0, c_12=0,
		c_20=0, c_21=0, c_22=0,
		c_30=0, c_31=0, c_32=0;

	// initial triangle

	a_0 = A[0+lda*0];
	a_1 = A[1+lda*0];
	a_2 = A[2+lda*0];
	a_3 = A[3+lda*0];
	
	b_0 = B[0+lda*0];
	
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;


	a_0 = A[0+lda*1];
	a_1 = A[1+lda*1];
	a_2 = A[2+lda*1];
	a_3 = A[3+lda*1];
	
	b_0 = B[0+lda*1];
	b_1 = B[1+lda*1];
	
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;

	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;


	a_0 = A[0+lda*2];
	a_1 = A[1+lda*2];
	a_2 = A[2+lda*2];
	a_3 = A[3+lda*2];
	
	b_0 = B[0+lda*2];
	b_1 = B[1+lda*2];
	b_2 = B[2+lda*2];
	
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;

	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;

	c_02 += a_0 * b_2;
	c_12 += a_1 * b_2;
	c_22 += a_2 * b_2;
	c_32 += a_3 * b_2;


	C[0+ldc*0] = c_00;
	C[1+ldc*0] = c_10;
	C[2+ldc*0] = c_20;
	C[3+ldc*0] = c_30;

	C[0+ldc*1] = c_01;
	C[1+ldc*1] = c_11;
	C[2+ldc*1] = c_21;
	C[3+ldc*1] = c_31;

	C[0+ldc*2] = c_02;
	C[1+ldc*2] = c_12;
	C[2+ldc*2] = c_22;
	C[3+ldc*2] = c_32;

	}



void corner_dtrmm_nt_u_4x2_lib4(double *A, double *B, double *C)
	{

	const int bs = 4;
	const int lda = bs;
	const int ldc = 4;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0,
		c_10=0, c_11=0,
		c_20=0, c_21=0,
		c_30=0, c_31=0;

	// initial triangle

	a_0 = A[0+lda*0];
	a_1 = A[1+lda*0];
	a_2 = A[2+lda*0];
	a_3 = A[3+lda*0];
	
	b_0 = B[0+lda*0];
	
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;


	a_0 = A[0+lda*1];
	a_1 = A[1+lda*1];
	a_2 = A[2+lda*1];
	a_3 = A[3+lda*1];
	
	b_0 = B[0+lda*1];
	b_1 = B[1+lda*1];
	
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;

	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;


	C[0+ldc*0] = c_00;
	C[1+ldc*0] = c_10;
	C[2+ldc*0] = c_20;
	C[3+ldc*0] = c_30;

	C[0+ldc*1] = c_01;
	C[1+ldc*1] = c_11;
	C[2+ldc*1] = c_21;
	C[3+ldc*1] = c_31;

	}



void corner_dtrmm_nt_u_4x1_lib4(double *A, double *B, double *C)
	{

	const int bs = 4;
	const int lda = bs;
	const int ldc = 4;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0,
		c_10=0,
		c_20=0,
		c_30=0;

	// initial triangle

	a_0 = A[0+lda*0];
	a_1 = A[1+lda*0];
	a_2 = A[2+lda*0];
	a_3 = A[3+lda*0];
	
	b_0 = B[0+lda*0];
	
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;


	C[0+ldc*0] = c_00;
	C[1+ldc*0] = c_10;
	C[2+ldc*0] = c_20;
	C[3+ldc*0] = c_30;

	}



void kernel_dtrmm_nt_l_4x4_lib4(int kmax, double *A, double *B, double *C)
	{
	
	const int lda = 4;
	const int ldc = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0, c_02=0, c_03=0,
		c_10=0, c_11=0, c_12=0, c_13=0,
		c_20=0, c_21=0, c_22=0, c_23=0,
		c_30=0, c_31=0, c_32=0, c_33=0;
		
	// kmax is multiple of bs
	for(k=0; k<kmax-4; k+=4)
		{
		
		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];
		
		b_0 = B[0+lda*0];
		b_1 = B[1+lda*0];
		b_2 = B[2+lda*0];
		b_3 = B[3+lda*0];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;
		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;
		c_23 += a_2 * b_3;
		c_33 += a_3 * b_3;


		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		a_2 = A[2+lda*1];
		a_3 = A[3+lda*1];
		
		b_0 = B[0+lda*1];
		b_1 = B[1+lda*1];
		b_2 = B[2+lda*1];
		b_3 = B[3+lda*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;
		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;
		c_23 += a_2 * b_3;
		c_33 += a_3 * b_3;


		a_0 = A[0+lda*2];
		a_1 = A[1+lda*2];
		a_2 = A[2+lda*2];
		a_3 = A[3+lda*2];
		
		b_0 = B[0+lda*2];
		b_1 = B[1+lda*2];
		b_2 = B[2+lda*2];
		b_3 = B[3+lda*2];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;
		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;
		c_23 += a_2 * b_3;
		c_33 += a_3 * b_3;


		a_0 = A[0+lda*3];
		a_1 = A[1+lda*3];
		a_2 = A[2+lda*3];
		a_3 = A[3+lda*3];
		
		b_0 = B[0+lda*3];
		b_1 = B[1+lda*3];
		b_2 = B[2+lda*3];
		b_3 = B[3+lda*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;
		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;
		c_23 += a_2 * b_3;
		c_33 += a_3 * b_3;
		
		
		A += 16;
		B += 16;

		}
		
	// final triangle

	// k=kmax-3
	a_0 = A[0+lda*0];
	a_1 = A[1+lda*0];
	a_2 = A[2+lda*0];
	a_3 = A[3+lda*0];
		
	b_0 = B[0+lda*0];
	b_1 = B[1+lda*0];
	b_2 = B[2+lda*0];
	b_3 = B[3+lda*0];
		
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;

	C[0+ldc*0] = c_00;
	C[1+ldc*0] = c_10;
	C[2+ldc*0] = c_20;
	C[3+ldc*0] = c_30;
		
	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;

	c_02 += a_0 * b_2;
	c_12 += a_1 * b_2;
	c_22 += a_2 * b_2;
	c_32 += a_3 * b_2;

	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;
	c_23 += a_2 * b_3;
	c_33 += a_3 * b_3;

	// k=kmax-2
	a_0 = A[0+lda*1];
	a_1 = A[1+lda*1];
	a_2 = A[2+lda*1];
	a_3 = A[3+lda*1];
		
	b_1 = B[1+lda*1];
	b_2 = B[2+lda*1];
	b_3 = B[3+lda*1];
		
	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;

	C[0+ldc*1] = c_01;
	C[1+ldc*1] = c_11;
	C[2+ldc*1] = c_21;
	C[3+ldc*1] = c_31;
		
	c_02 += a_0 * b_2;
	c_12 += a_1 * b_2;
	c_22 += a_2 * b_2;
	c_32 += a_3 * b_2;

	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;
	c_23 += a_2 * b_3;
	c_33 += a_3 * b_3;

	// k=kmax-1
	a_0 = A[0+lda*2];
	a_1 = A[1+lda*2];
	a_2 = A[2+lda*2];
	a_3 = A[3+lda*2];
		
	b_2 = B[2+lda*2];
	b_3 = B[3+lda*2];

	c_02 += a_0 * b_2;
	c_12 += a_1 * b_2;
	c_22 += a_2 * b_2;
	c_32 += a_3 * b_2;

	C[0+ldc*2] = c_02;
	C[1+ldc*2] = c_12;
	C[2+ldc*2] = c_22;
	C[3+ldc*2] = c_32;
		
	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;
	c_23 += a_2 * b_3;
	c_33 += a_3 * b_3;

	// k=kmax
	a_0 = A[0+lda*3];
	a_1 = A[1+lda*3];
	a_2 = A[2+lda*3];
	a_3 = A[3+lda*3];
		
	b_3 = B[3+lda*3];

	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;
	c_23 += a_2 * b_3;
	c_33 += a_3 * b_3;

	C[0+ldc*3] = c_03;
	C[1+ldc*3] = c_13;
	C[2+ldc*3] = c_23;
	C[3+ldc*3] = c_33;
		
	}



void kernel_dtrmm_nt_l_2x4_lib4(int kmax, double *A, double *B, double *C)
	{
	
	const int lda = 4;
	const int ldc = 4;

	int k;

	double
		a_0, a_1,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0, c_02=0, c_03=0,
		c_10=0, c_11=0, c_12=0, c_13=0;
		
	// kmax is multiple of bs
	for(k=0; k<kmax-4; k+=4)
		{
		
		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		
		b_0 = B[0+lda*0];
		b_1 = B[1+lda*0];
		b_2 = B[2+lda*0];
		b_3 = B[3+lda*0];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;


		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		
		b_0 = B[0+lda*1];
		b_1 = B[1+lda*1];
		b_2 = B[2+lda*1];
		b_3 = B[3+lda*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;


		a_0 = A[0+lda*2];
		a_1 = A[1+lda*2];
		
		b_0 = B[0+lda*2];
		b_1 = B[1+lda*2];
		b_2 = B[2+lda*2];
		b_3 = B[3+lda*2];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;


		a_0 = A[0+lda*3];
		a_1 = A[1+lda*3];
		
		b_0 = B[0+lda*3];
		b_1 = B[1+lda*3];
		b_2 = B[2+lda*3];
		b_3 = B[3+lda*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;
		
		
		A += 16;
		B += 16;

		}
		
	// final triangle

	// k=kmax-3
	a_0 = A[0+lda*0];
	a_1 = A[1+lda*0];
		
	b_0 = B[0+lda*0];
	b_1 = B[1+lda*0];
	b_2 = B[2+lda*0];
	b_3 = B[3+lda*0];
		
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;

	C[0+ldc*0] = c_00;
	C[1+ldc*0] = c_10;
		
	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;

	c_02 += a_0 * b_2;
	c_12 += a_1 * b_2;

	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;

	// k=kmax-2
	a_0 = A[0+lda*1];
	a_1 = A[1+lda*1];
		
	b_1 = B[1+lda*1];
	b_2 = B[2+lda*1];
	b_3 = B[3+lda*1];
		
	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;

	C[0+ldc*1] = c_01;
	C[1+ldc*1] = c_11;
		
	c_02 += a_0 * b_2;
	c_12 += a_1 * b_2;

	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;

	// k=kmax-1
	a_0 = A[0+lda*2];
	a_1 = A[1+lda*2];
		
	b_2 = B[2+lda*2];
	b_3 = B[3+lda*2];

	c_02 += a_0 * b_2;
	c_12 += a_1 * b_2;

	C[0+ldc*2] = c_02;
	C[1+ldc*2] = c_12;
		
	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;

	// k=kmax
	a_0 = A[0+lda*3];
	a_1 = A[1+lda*3];
		
	b_3 = B[3+lda*3];

	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;

	C[0+ldc*3] = c_03;
	C[1+ldc*3] = c_13;
		
	}



void kernel_dtrmm_nt_l_4x2_lib4(int kmax, double *A, double *B, double *C)
	{
	
	const int lda = 4;
	const int ldc = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1,
		c_00=0, c_01=0,
		c_10=0, c_11=0,
		c_20=0, c_21=0,
		c_30=0, c_31=0;
		
	// kmax is multiple of bs
	for(k=0; k<kmax-4; k+=4)
		{
		
		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];
		
		b_0 = B[0+lda*0];
		b_1 = B[1+lda*0];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		a_2 = A[2+lda*1];
		a_3 = A[3+lda*1];
		
		b_0 = B[0+lda*1];
		b_1 = B[1+lda*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		a_0 = A[0+lda*2];
		a_1 = A[1+lda*2];
		a_2 = A[2+lda*2];
		a_3 = A[3+lda*2];
		
		b_0 = B[0+lda*2];
		b_1 = B[1+lda*2];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		a_0 = A[0+lda*3];
		a_1 = A[1+lda*3];
		a_2 = A[2+lda*3];
		a_3 = A[3+lda*3];
		
		b_0 = B[0+lda*3];
		b_1 = B[1+lda*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;
		
		
		A += 16;
		B += 16;

		}
		
	// final triangle

	// k=kmax-3
	a_0 = A[0+lda*0];
	a_1 = A[1+lda*0];
	a_2 = A[2+lda*0];
	a_3 = A[3+lda*0];
		
	b_0 = B[0+lda*0];
	b_1 = B[1+lda*0];
		
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;

	C[0+ldc*0] = c_00;
	C[1+ldc*0] = c_10;
	C[2+ldc*0] = c_20;
	C[3+ldc*0] = c_30;
		
	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;

	// k=kmax-2
	a_0 = A[0+lda*1];
	a_1 = A[1+lda*1];
	a_2 = A[2+lda*1];
	a_3 = A[3+lda*1];
		
	b_1 = B[1+lda*1];
		
	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;

	C[0+ldc*1] = c_01;
	C[1+ldc*1] = c_11;
	C[2+ldc*1] = c_21;
	C[3+ldc*1] = c_31;
	
	}



void kernel_dtrmm_nt_l_2x2_lib4(int kmax, double *A, double *B, double *C)
	{
	
	const int lda = 4;
	const int ldc = 4;

	int k;

	double
		a_0, a_1,
		b_0, b_1,
		c_00=0, c_01=0,
		c_10=0, c_11=0;
		
	// kmax is multiple of bs
	for(k=0; k<kmax-4; k+=4)
		{
		
		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		
		b_0 = B[0+lda*0];
		b_1 = B[1+lda*0];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;


		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		
		b_0 = B[0+lda*1];
		b_1 = B[1+lda*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;


		a_0 = A[0+lda*2];
		a_1 = A[1+lda*2];
		
		b_0 = B[0+lda*2];
		b_1 = B[1+lda*2];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;


		a_0 = A[0+lda*3];
		a_1 = A[1+lda*3];
		
		b_0 = B[0+lda*3];
		b_1 = B[1+lda*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		
		
		A += 16;
		B += 16;

		}
		
	// final triangle

	// k=kmax-3
	a_0 = A[0+lda*0];
	a_1 = A[1+lda*0];
		
	b_0 = B[0+lda*0];
	b_1 = B[1+lda*0];
		
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;

	C[0+ldc*0] = c_00;
	C[1+ldc*0] = c_10;
		
	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;

	// k=kmax-2
	a_0 = A[0+lda*1];
	a_1 = A[1+lda*1];
		
	b_1 = B[1+lda*1];
		
	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;

	C[0+ldc*1] = c_01;
	C[1+ldc*1] = c_11;
	
	}



// A upper triangle matrix on the left
void kernel_dtrmm_l_u_nt_4x4_lib4(int kmax, double *A, double *B, double *C, double *D, int alg)
	{
	
	const int lda = 4;
	const int ldc = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0, c_02=0, c_03=0,
		c_10=0, c_11=0, c_12=0, c_13=0,
		c_20=0, c_21=0, c_22=0, c_23=0,
		c_30=0, c_31=0, c_32=0, c_33=0;

	// triangle at the beginning

	// k = 0
	a_0 = A[0+lda*0];
		
	b_0 = B[0+lda*0];
	b_1 = B[1+lda*0];
	b_2 = B[2+lda*0];
	b_3 = B[3+lda*0];
		
	c_00 += a_0 * b_0;

	c_01 += a_0 * b_1;

	c_02 += a_0 * b_2;

	c_03 += a_0 * b_3;


	// k = 1
	a_0 = A[0+lda*1];
	a_1 = A[1+lda*1];
		
	b_0 = B[0+lda*1];
	b_1 = B[1+lda*1];
	b_2 = B[2+lda*1];
	b_3 = B[3+lda*1];
		
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;

	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;

	c_02 += a_0 * b_2;
	c_12 += a_1 * b_2;

	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;


	// k = 2
	a_0 = A[0+lda*2];
	a_1 = A[1+lda*2];
	a_2 = A[2+lda*2];
		
	b_0 = B[0+lda*2];
	b_1 = B[1+lda*2];
	b_2 = B[2+lda*2];
	b_3 = B[3+lda*2];
		
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;

	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;

	c_02 += a_0 * b_2;
	c_12 += a_1 * b_2;
	c_22 += a_2 * b_2;

	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;
	c_23 += a_2 * b_3;


	// k = 3
	a_0 = A[0+lda*3];
	a_1 = A[1+lda*3];
	a_2 = A[2+lda*3];
	a_3 = A[3+lda*3];
		
	b_0 = B[0+lda*3];
	b_1 = B[1+lda*3];
	b_2 = B[2+lda*3];
	b_3 = B[3+lda*3];
		
	c_00 += a_0 * b_0;
	c_10 += a_1 * b_0;
	c_20 += a_2 * b_0;
	c_30 += a_3 * b_0;

	c_01 += a_0 * b_1;
	c_11 += a_1 * b_1;
	c_21 += a_2 * b_1;
	c_31 += a_3 * b_1;

	c_02 += a_0 * b_2;
	c_12 += a_1 * b_2;
	c_22 += a_2 * b_2;
	c_32 += a_3 * b_2;

	c_03 += a_0 * b_3;
	c_13 += a_1 * b_3;
	c_23 += a_2 * b_3;
	c_33 += a_3 * b_3;

	k=4;
	A += 16;
	B += 16;

	// kmax is multiple of bs
	for(; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];
		
		b_0 = B[0+lda*0];
		b_1 = B[1+lda*0];
		b_2 = B[2+lda*0];
		b_3 = B[3+lda*0];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;
		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;
		c_23 += a_2 * b_3;
		c_33 += a_3 * b_3;


		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		a_2 = A[2+lda*1];
		a_3 = A[3+lda*1];
		
		b_0 = B[0+lda*1];
		b_1 = B[1+lda*1];
		b_2 = B[2+lda*1];
		b_3 = B[3+lda*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;
		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;
		c_23 += a_2 * b_3;
		c_33 += a_3 * b_3;
		

		a_0 = A[0+lda*2];
		a_1 = A[1+lda*2];
		a_2 = A[2+lda*2];
		a_3 = A[3+lda*2];
		
		b_0 = B[0+lda*2];
		b_1 = B[1+lda*2];
		b_2 = B[2+lda*2];
		b_3 = B[3+lda*2];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;
		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;
		c_23 += a_2 * b_3;
		c_33 += a_3 * b_3;
		

		a_0 = A[0+lda*3];
		a_1 = A[1+lda*3];
		a_2 = A[2+lda*3];
		a_3 = A[3+lda*3];
		
		b_0 = B[0+lda*3];
		b_1 = B[1+lda*3];
		b_2 = B[2+lda*3];
		b_3 = B[3+lda*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;
		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;
		c_23 += a_2 * b_3;
		c_33 += a_3 * b_3;
		
		
		A += 16;
		B += 16;

		}
	for(; k<kmax; k++)
		{
		
		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];
		
		b_0 = B[0+lda*0];
		b_1 = B[1+lda*0];
		b_2 = B[2+lda*0];
		b_3 = B[3+lda*0];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_02 += a_0 * b_2;
		c_12 += a_1 * b_2;
		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_03 += a_0 * b_3;
		c_13 += a_1 * b_3;
		c_23 += a_2 * b_3;
		c_33 += a_3 * b_3;
		
		A += 4;
		B += 4;

		}
	
	double
		d_00, d_01, d_02, d_03,
		d_10, d_11, d_12, d_13,
		d_20, d_21, d_22, d_23,
		d_30, d_31, d_32, d_33;

	// store
	if(alg==0)
		{
		D[0+ldc*0] = c_00;
		D[1+ldc*0] = c_10;
		D[2+ldc*0] = c_20;
		D[3+ldc*0] = c_30;

		D[0+ldc*1] = c_01;
		D[1+ldc*1] = c_11;
		D[2+ldc*1] = c_21;
		D[3+ldc*1] = c_31;

		D[0+ldc*2] = c_02;
		D[1+ldc*2] = c_12;
		D[2+ldc*2] = c_22;
		D[3+ldc*2] = c_32;

		D[0+ldc*3] = c_03;
		D[1+ldc*3] = c_13;
		D[2+ldc*3] = c_23;
		D[3+ldc*3] = c_33;
		}
	else
		{
		d_00 = C[0+ldc*0];
		d_10 = C[1+ldc*0];
		d_20 = C[2+ldc*0];
		d_30 = C[3+ldc*0];
			
		d_01 = C[0+ldc*1];
		d_11 = C[1+ldc*1];
		d_21 = C[2+ldc*1];
		d_31 = C[3+ldc*1];
			
		d_02 = C[0+ldc*2];
		d_12 = C[1+ldc*2];
		d_22 = C[2+ldc*2];
		d_32 = C[3+ldc*2];
			
		d_03 = C[0+ldc*3];
		d_13 = C[1+ldc*3];
		d_23 = C[2+ldc*3];
		d_33 = C[3+ldc*3];
			
		if(alg==1)
			{
			d_00 += c_00;
			d_10 += c_10;
			d_20 += c_20;
			d_30 += c_30;

			d_01 += c_01;
			d_11 += c_11;
			d_21 += c_21;
			d_31 += c_31;

			d_02 += c_02;
			d_12 += c_12;
			d_22 += c_22;
			d_32 += c_32;

			d_03 += c_03;
			d_13 += c_13;
			d_23 += c_23;
			d_33 += c_33;
			}
		else
			{
			d_00 -= c_00;
			d_10 -= c_10;
			d_20 -= c_20;
			d_30 -= c_30;

			d_01 -= c_01;
			d_11 -= c_11;
			d_21 -= c_21;
			d_31 -= c_31;

			d_02 -= c_02;
			d_12 -= c_12;
			d_22 -= c_22;
			d_32 -= c_32;

			d_03 -= c_03;
			d_13 -= c_13;
			d_23 -= c_23;
			d_33 -= c_33;
			}

		D[0+ldc*0] = d_00;
		D[1+ldc*0] = d_10;
		D[2+ldc*0] = d_20;
		D[3+ldc*0] = d_30;

		D[0+ldc*1] = d_01;
		D[1+ldc*1] = d_11;
		D[2+ldc*1] = d_21;
		D[3+ldc*1] = d_31;

		D[0+ldc*2] = d_02;
		D[1+ldc*2] = d_12;
		D[2+ldc*2] = d_22;
		D[3+ldc*2] = d_32;

		D[0+ldc*3] = d_03;
		D[1+ldc*3] = d_13;
		D[2+ldc*3] = d_23;
		D[3+ldc*3] = d_33;

		}

	
	}




// rank 3 update
void corner_dtrmm_l_u_nt_3x4_lib4(double *A, double *B, int alg, double *C, double *D)
	{

	const int bs = 4;

	double
		a_0, a_1, a_2,
		b_0,
		d_0, d_4, d_8, d_c,
		d_1, d_5, d_9, d_d,
		d_2, d_6, d_a, d_e;
	

	// k = 0
	a_0 = A[0];
	a_1 = A[1];
	a_2 = A[2];

	b_0 = B[0];
	d_0 = a_0*b_0;
	d_1 = a_1*b_0;
	d_2 = a_2*b_0;
	b_0 = B[1];
	d_4 = a_0*b_0;
	d_5 = a_1*b_0;
	d_6 = a_2*b_0;
	b_0 = B[2];
	d_8 = a_0*b_0;
	d_9 = a_1*b_0;
	d_a = a_2*b_0;
	b_0 = B[3];
	d_c = a_0*b_0;
	d_d = a_1*b_0;
	d_e = a_2*b_0;


	// k = 1
	a_0 = A[4];
	a_1 = A[5];
	a_2 = A[6];

	b_0 = B[4];
	d_0 += a_0*b_0;
	d_1 += a_1*b_0;
	d_2 += a_2*b_0;
	b_0 = B[5];
	d_4 += a_0*b_0;
	d_5 += a_1*b_0;
	d_6 += a_2*b_0;
	b_0 = B[6];
	d_8 += a_0*b_0;
	d_9 += a_1*b_0;
	d_a += a_2*b_0;
	b_0 = B[7];
	d_c += a_0*b_0;
	d_d += a_1*b_0;
	d_e += a_2*b_0;


	// k = 2
	a_0 = A[8];
	a_1 = A[9];
	a_2 = A[10];

	b_0 = B[8];
	d_0 += a_0*b_0;
	d_1 += a_1*b_0;
	d_2 += a_2*b_0;
	b_0 = B[9];
	d_4 += a_0*b_0;
	d_5 += a_1*b_0;
	d_6 += a_2*b_0;
	b_0 = B[10];
	d_8 += a_0*b_0;
	d_9 += a_1*b_0;
	d_a += a_2*b_0;
	b_0 = B[11];
	d_c += a_0*b_0;
	d_d += a_1*b_0;
	d_e += a_2*b_0;



	if(alg!=0)
		{
		if(alg==1)
			{
			d_0 = C[0+bs*0] + d_0;
			d_1 = C[1+bs*0] + d_1;
			d_2 = C[2+bs*0] + d_2;
			d_4 = C[0+bs*1] + d_4;
			d_5 = C[1+bs*1] + d_5;
			d_6 = C[2+bs*1] + d_6;
			d_8 = C[0+bs*2] + d_8;
			d_9 = C[1+bs*2] + d_9;
			d_a = C[2+bs*2] + d_a;
			d_c = C[0+bs*3] + d_c;
			d_d = C[1+bs*3] + d_d;
			d_e = C[2+bs*3] + d_e;
			}
		else
			{
			d_0 = C[0+bs*0] - d_0;
			d_1 = C[1+bs*0] - d_1;
			d_2 = C[2+bs*0] - d_2;
			d_4 = C[0+bs*1] - d_4;
			d_5 = C[1+bs*1] - d_5;
			d_6 = C[2+bs*1] - d_6;
			d_8 = C[0+bs*2] - d_8;
			d_9 = C[1+bs*2] - d_9;
			d_a = C[2+bs*2] - d_a;
			d_c = C[0+bs*3] - d_c;
			d_d = C[1+bs*3] - d_d;
			d_e = C[2+bs*3] - d_e;
			}
		}
	
	D[0+bs*0] = d_0;
	D[1+bs*0] = d_1;
	D[2+bs*0] = d_2;
	D[0+bs*1] = d_4;
	D[1+bs*1] = d_5;
	D[2+bs*1] = d_6;
	D[0+bs*2] = d_8;
	D[1+bs*2] = d_9;
	D[2+bs*2] = d_a;
	D[0+bs*3] = d_c;
	D[1+bs*3] = d_d;
	D[2+bs*3] = d_e;

	}



// rank 2 update
void corner_dtrmm_l_u_nt_2x4_lib4(double *A, double *B, int alg, double *C, double *D)
	{

	const int bs = 4;

	double
		a_0, a_1,
		b_0,
		d_0, d_4, d_8, d_c,
		d_1, d_5, d_9, d_d;
	

	// k = 0
	a_0 = A[0];
	a_1 = A[1];

	b_0 = B[0];
	d_0 = a_0*b_0;
	d_1 = a_1*b_0;
	b_0 = B[1];
	d_4 = a_0*b_0;
	d_5 = a_1*b_0;
	b_0 = B[2];
	d_8 = a_0*b_0;
	d_9 = a_1*b_0;
	b_0 = B[3];
	d_c = a_0*b_0;
	d_d = a_1*b_0;


	// k = 0
	a_0 = A[4];
	a_1 = A[5];

	b_0 = B[4];
	d_0 += a_0*b_0;
	d_1 += a_1*b_0;
	b_0 = B[5];
	d_4 += a_0*b_0;
	d_5 += a_1*b_0;
	b_0 = B[6];
	d_8 += a_0*b_0;
	d_9 += a_1*b_0;
	b_0 = B[7];
	d_c += a_0*b_0;
	d_d += a_1*b_0;


	if(alg!=0)
		{
		if(alg==1)
			{
			d_0 = C[0+bs*0] + d_0;
			d_1 = C[1+bs*0] + d_1;
			d_4 = C[0+bs*1] + d_4;
			d_5 = C[1+bs*1] + d_5;
			d_8 = C[0+bs*2] + d_8;
			d_9 = C[1+bs*2] + d_9;
			d_c = C[0+bs*3] + d_c;
			d_d = C[1+bs*3] + d_d;
			}
		else
			{
			d_0 = C[0+bs*0] - d_0;
			d_1 = C[1+bs*0] - d_1;
			d_4 = C[0+bs*1] - d_4;
			d_5 = C[1+bs*1] - d_5;
			d_8 = C[0+bs*2] - d_8;
			d_9 = C[1+bs*2] - d_9;
			d_c = C[0+bs*3] - d_c;
			d_d = C[1+bs*3] - d_d;
			}
		}
	
	D[0+bs*0] = d_0;
	D[1+bs*0] = d_1;
	D[0+bs*1] = d_4;
	D[1+bs*1] = d_5;
	D[0+bs*2] = d_8;
	D[1+bs*2] = d_9;
	D[0+bs*3] = d_c;
	D[1+bs*3] = d_d;

	}



// rank 1 update
void corner_dtrmm_l_u_nt_1x4_lib4(double *A, double *B, int alg, double *C, double *D)
	{

	const int bs = 4;

	double
		a_0,
		b_0,
		d_0, d_1, d_2, d_3;
	

	a_0 = A[0];

	b_0 = B[0];
	d_0 = a_0*b_0;
	b_0 = B[1];
	d_1 = a_0*b_0;
	b_0 = B[2];
	d_2 = a_0*b_0;
	b_0 = B[3];
	d_3 = a_0*b_0;


	if(alg!=0)
		{
		if(alg==1)
			{
			d_0 = C[0+bs*0] + d_0;
			d_1 = C[0+bs*1] + d_1;
			d_2 = C[0+bs*2] + d_2;
			d_3 = C[0+bs*3] + d_3;
			}
		else
			{
			d_0 = C[0+bs*0] - d_0;
			d_1 = C[0+bs*1] - d_1;
			d_2 = C[0+bs*2] - d_2;
			d_3 = C[0+bs*3] - d_3;
			}
		}
	
	D[0+bs*0] = d_0;
	D[0+bs*1] = d_1;
	D[0+bs*2] = d_2;
	D[0+bs*3] = d_3;

	}




