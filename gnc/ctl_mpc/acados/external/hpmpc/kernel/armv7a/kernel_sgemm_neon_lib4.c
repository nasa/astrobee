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



// normal-transposed, 12x4 with data packed in 4
void kernel_sgemm_nt_12x4_lib4(int kmax, float *A0, float *A1, float *A2, float *B, float *C0, float *C1, float *C2, float *D0, float *D1, float *D2, int alg)
	{
	
	if(kmax<=0)
		return;
		
	__builtin_prefetch( A0 );
	__builtin_prefetch( A1 );
	__builtin_prefetch( A2 );
	__builtin_prefetch( B  );

	int k_iter = kmax/4;
	int k_left = kmax%4;
	
//	printf("\n%d %d %d\n", kmax, k_iter, k_left);

	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"pld    [%2, #64]                \n\t" // prefetch A0 to L1
		"pld    [%3, #64]                \n\t" // prefetch A1 to L1
		"pld    [%4, #64]                \n\t" // prefetch A2 to L1
		"pld    [%5, #64]                \n\t" // prefetch B to L1
		"                                \n\t"
		"                                \n\t"
		"mov    r0, %0                   \n\t" // k_iter
		"                                \n\t"
		"                                \n\t"
		"vld1.64   {d0, d1}, [%5:128]! \n\t" // load B to registers
		"vld1.64   {d2, d3}, [%2:128]! \n\t" // load A0 to registers
		"vld1.64   {d4, d5}, [%3:128]! \n\t" // load A1
/*		"vld1.64   {d6, d7}, [%4:128]! \n\t" // load A1*/
		"                                \n\t"
		"cmp    r0, #0                   \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"ldr    r4, %5                   \n\t" // load address of C*/
/*		"ldr    r5, %6                   \n\t" // load address of C*/
/*		"ldr    r6, %7                   \n\t" // alg*/
		"                                \n\t"
		"                                \n\t"
		"vldr   d8, .DZERO_12x4          \n\t" // load zero double
		"vldr   d9, .DZERO_12x4+8        \n\t" // load zero double
		"vmov   q5, q4                   \n\t"
		"vmov   q6, q4                   \n\t"
		"vmov   q7, q4                   \n\t"
		"vmov   q8, q4                   \n\t"
		"vmov   q9, q4                   \n\t"
		"vmov   q10, q4                  \n\t"
		"vmov   q11, q4                  \n\t"
		"vmov   q12, q4                  \n\t"
		"vmov   q13, q4                  \n\t"
		"vmov   q14, q4                  \n\t"
		"vmov   q15, q4                  \n\t"
		"                                \n\t"
		"                                \n\t"
		"ble    .DCONSIDERLEFT2_12x4           \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DLOOPKITER_12x4:                    \n\t" // main loop
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q4, q1, d0[0]        \n\t"
		"vldr   d6, [%4, #0]             \n\t"
		"vmla.f32  q5, q1, d0[1]        \n\t"
		"vldr   d7, [%4, #8]             \n\t"
		"vmla.f32  q6, q1, d1[0]        \n\t"
		"pld    [%2, #128]                \n\t" // A0
		"vmla.f32  q7, q1, d1[1]        \n\t"
		"vldr   d2, [%2, #0]             \n\t"
		"                                \n\t"
		"vmla.f32  q8, q2, d0[0]        \n\t"
		"vldr   d3, [%2, #8]             \n\t"
		"vmla.f32  q9, q2, d0[1]        \n\t"
		"pld    [%3, #128]                \n\t" // A1
		"vmla.f32  q10, q2, d1[0]        \n\t"
		"pld    [%4, #144]                \n\t" // A2
		"vmla.f32  q11, q2, d1[1]        \n\t"
		"vldr   d4, [%5, #0]             \n\t"
		"                                \n\t"
		"vmla.f32  q12, q3, d0[0]        \n\t"
		"vldr   d5, [%5, #8]             \n\t"
		"vmla.f32  q13, q3, d0[1]        \n\t"
		"vldr   d0, [%3, #0]             \n\t"
		"vmla.f32  q14, q3, d1[0]        \n\t"
		"pld    [%5, #128]                \n\t" // B
		"vmla.f32  q15, q3, d1[1]        \n\t"
		"vldr   d1, [%3, #8]             \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q4, q1, d4[0]        \n\t"
		"vldr   d6, [%4, #16]             \n\t"
		"vmla.f32  q5, q1, d4[1]        \n\t"
		"vldr   d7, [%4, #24]             \n\t"
		"vmla.f32  q6, q1, d5[0]        \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"vmla.f32  q7, q1, d5[1]        \n\t"
		"vldr   d2, [%2, #16]             \n\t"
		"                                \n\t"
		"vmla.f32  q8, q0, d4[0]        \n\t"
		"vldr   d3, [%2, #24]             \n\t"
		"vmla.f32  q9, q0, d4[1]        \n\t"
		"vmla.f32  q10, q0, d5[0]        \n\t"
		"vmla.f32  q11, q0, d5[1]        \n\t"
		"vldr   d0, [%5, #16]             \n\t"
		"                                \n\t"
		"vmla.f32  q12, q3, d4[0]        \n\t"
		"vldr   d1, [%5, #24]             \n\t"
		"vmla.f32  q13, q3, d4[1]        \n\t"
		"vldr   d4, [%3, #16]             \n\t"
		"vmla.f32  q14, q3, d5[0]        \n\t"
		"cmp    r0, #0                   \n\t" // next iter?
		"vmla.f32  q15, q3, d5[1]        \n\t"
		"vldr   d5, [%3, #24]             \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q4, q1, d0[0]        \n\t"
		"vldr   d6, [%4, #32]             \n\t"
		"vmla.f32  q5, q1, d0[1]        \n\t"
		"vldr   d7, [%4, #40]             \n\t"
		"vmla.f32  q6, q1, d1[0]        \n\t"
		"vmla.f32  q7, q1, d1[1]        \n\t"
		"vldr   d2, [%2, #32]             \n\t"
		"                                \n\t"
		"vmla.f32  q8, q2, d0[0]        \n\t"
		"vldr   d3, [%2, #40]             \n\t"
		"vmla.f32  q9, q2, d0[1]        \n\t"
		"vmla.f32  q10, q2, d1[0]        \n\t"
		"vmla.f32  q11, q2, d1[1]        \n\t"
		"vldr   d4, [%5, #32]             \n\t"
		"                                \n\t"
		"vmla.f32  q12, q3, d0[0]        \n\t"
		"vldr   d5, [%5, #40]             \n\t"
		"vmla.f32  q13, q3, d0[1]        \n\t"
		"vldr   d0, [%3, #32]             \n\t"
		"vmla.f32  q14, q3, d1[0]        \n\t"
		"vmla.f32  q15, q3, d1[1]        \n\t"
		"vldr   d1, [%3, #40]             \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q4, q1, d4[0]        \n\t"
		"vldr   d6, [%4, #48]             \n\t"
		"vmla.f32  q5, q1, d4[1]        \n\t"
		"vldr   d7, [%4, #56]             \n\t"
		"vmla.f32  q6, q1, d5[0]        \n\t"
		"add    %4, %4, #64              \n\t" // increase B
		"vmla.f32  q7, q1, d5[1]        \n\t"
		"vldr   d2, [%2, #48]             \n\t"
		"                                \n\t"
		"vmla.f32  q8, q0, d4[0]        \n\t"
		"vldr   d3, [%2, #56]             \n\t"
		"vmla.f32  q9, q0, d4[1]        \n\t"
		"add    %2, %2, #64              \n\t" // increase B
		"vmla.f32  q10, q0, d5[0]        \n\t"
		"vmla.f32  q11, q0, d5[1]        \n\t"
		"vldr   d0, [%5, #48]             \n\t"
		"                                \n\t"
		"vmla.f32  q12, q3, d4[0]        \n\t"
		"vldr   d1, [%5, #56]             \n\t"
		"vmla.f32  q13, q3, d4[1]        \n\t"
		"vldr   d4, [%3, #48]             \n\t"
		"vmla.f32  q14, q3, d5[0]        \n\t"
		"add    %5, %5, #64              \n\t" // increase B
		"vmla.f32  q15, q3, d5[1]        \n\t"
		"vldr   d5, [%3, #56]             \n\t"
		"add    %3, %3, #64              \n\t" // increase B
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"bgt    .DLOOPKITER_12x4              \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERLEFT2_12x4:                 \n\t" // consider left k+=2
		"                                \n\t"
		"mov    r0, %1                   \n\t" // k_left
		"cmp    r0, #1                   \n\t"
		"ble    .DCONSIDERLEFT1_12x4          \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q4, q1, d0[0]        \n\t"
		"vldr   d6, [%4, #0]             \n\t"
		"vmla.f32  q5, q1, d0[1]        \n\t"
		"vldr   d7, [%4, #8]             \n\t"
		"vmla.f32  q6, q1, d1[0]        \n\t"
		"vmla.f32  q7, q1, d1[1]        \n\t"
		"vldr   d2, [%2, #0]             \n\t"
		"                                \n\t"
		"vmla.f32  q8, q2, d0[0]        \n\t"
		"vldr   d3, [%2, #8]             \n\t"
		"vmla.f32  q9, q2, d0[1]        \n\t"
		"vmla.f32  q10, q2, d1[0]        \n\t"
		"vmla.f32  q11, q2, d1[1]        \n\t"
		"vldr   d4, [%5, #0]             \n\t"
		"                                \n\t"
		"vmla.f32  q12, q3, d0[0]        \n\t"
		"vldr   d5, [%5, #8]             \n\t"
		"vmla.f32  q13, q3, d0[1]        \n\t"
		"vldr   d0, [%3, #0]             \n\t"
		"vmla.f32  q14, q3, d1[0]        \n\t"
		"vmla.f32  q15, q3, d1[1]        \n\t"
		"vldr   d1, [%3, #8]             \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q4, q1, d4[0]        \n\t"
		"vldr   d6, [%4, #16]             \n\t"
		"vmla.f32  q5, q1, d4[1]        \n\t"
		"vldr   d7, [%4, #24]             \n\t"
		"vmla.f32  q6, q1, d5[0]        \n\t"
		"add    %4, %4, #32              \n\t" // increase B
		"vmla.f32  q7, q1, d5[1]        \n\t"
		"vldr   d2, [%2, #16]             \n\t"
		"                                \n\t"
		"vmla.f32  q8, q0, d4[0]        \n\t"
		"vldr   d3, [%2, #24]             \n\t"
		"vmla.f32  q9, q0, d4[1]        \n\t"
		"vmla.f32  q10, q0, d5[0]        \n\t"
		"vmla.f32  q11, q0, d5[1]        \n\t"
		"vldr   d0, [%5, #16]             \n\t"
		"                                \n\t"
		"vmla.f32  q12, q3, d4[0]        \n\t"
		"vldr   d1, [%5, #24]             \n\t"
		"vmla.f32  q13, q3, d4[1]        \n\t"
		"vldr   d4, [%3, #16]             \n\t"
		"vmla.f32  q14, q3, d5[0]        \n\t"
		"vmla.f32  q15, q3, d5[1]        \n\t"
		"vldr   d5, [%3, #24]             \n\t"
		"                                \n\t"
		"sub    r0, r0, #2               \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERLEFT1_12x4:                 \n\t" // consider left k++
		"                                \n\t"
		"cmp    r0, #0                   \n\t"
		"ble    .DPOSTACCUM_12x4              \n\t"
		"                                \n\t"
		"vmla.f32  q4, q1, d0[0]        \n\t"
		"vldr   d6, [%4, #0]             \n\t"
		"vmla.f32  q5, q1, d0[1]        \n\t"
		"vldr   d7, [%4, #8]             \n\t"
		"vmla.f32  q6, q1, d1[0]        \n\t"
		"vmla.f32  q7, q1, d1[1]        \n\t"
		"                                \n\t"
		"vmla.f32  q8, q2, d0[0]        \n\t"
		"vmla.f32  q9, q2, d0[1]        \n\t"
		"vmla.f32  q10, q2, d1[0]        \n\t"
		"vmla.f32  q11, q2, d1[1]        \n\t"
		"                                \n\t"
		"vmla.f32  q12, q3, d0[0]        \n\t"
		"vmla.f32  q13, q3, d0[1]        \n\t"
		"vmla.f32  q14, q3, d1[0]        \n\t"
		"vmla.f32  q15, q3, d1[1]        \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DPOSTACCUM_12x4:                    \n\t"
		"                                \n\t"
		"cmp    %12, #0                   \n\t"
		"beq    .D0_12x4                      \n\t" // if alg==0, jump
		"                                \n\t"// alg==-1
		"ldr    r0, %6                   \n\t" // load address of C0
		"ldr    r1, %7                   \n\t" // load address of C1
		"ldr    r2, %8                   \n\t" // load address of C2
		"                                \n\t"
		"pld    [r0, #0]                \n\t" // C0
		"pld    [r1, #0]                \n\t" // C1
		"pld    [r2, #0]                \n\t" // C2
		"                                \n\t"
		"cmp    %12, #1                   \n\t"
		"beq    .D1_12x4                      \n\t" // if alg==1, jump
		"                                \n\t"
		"vld1.64   {d0, d1, d2, d3},   [r0:128]! \n\t" // load C0
		"vld1.64   {d4, d5, d6, d7},   [r0:128]  \n\t" // load C0
		"vsub.f32  q4, q0, q4            \n\t"
		"vsub.f32  q5, q1, q5            \n\t"
		"vld1.64   {d0, d1, d2, d3},   [r1:128]! \n\t" // load C0
		"vsub.f32  q6, q2, q6            \n\t"
		"vsub.f32  q7, q3, q7            \n\t"
		"vld1.64   {d4, d5, d6, d7},   [r1:128]  \n\t" // load C0
		"vsub.f32  q8, q0, q8            \n\t"
		"vsub.f32  q9, q1, q9            \n\t"
		"vld1.64   {d0, d1, d2, d3},   [r2:128]! \n\t" // load C0
		"vsub.f32  q10, q2, q10            \n\t"
		"vsub.f32  q11, q3, q11            \n\t"
		"vld1.64   {d4, d5, d6, d7},   [r2:128]  \n\t" // load C0
		"vsub.f32  q12, q0, q12            \n\t"
		"vsub.f32  q13, q1, q13            \n\t"
		"vsub.f32  q14, q2, q14            \n\t"
		"vsub.f32  q15, q3, q15            \n\t"

		"b      .D0_12x4                      \n\t" // jump to end
		"                                \n\t"
		"                                \n\t"
		".D1_12x4:                            \n\t" // alg==1
		"                                \n\t"
		"vld1.64   {d0, d1, d2, d3},   [r0:128]! \n\t" // load C0
		"vld1.64   {d4, d5, d6, d7},   [r0:128]  \n\t" // load C0
		"vadd.f32  q4, q0, q4            \n\t"
		"vadd.f32  q5, q1, q5            \n\t"
		"vld1.64   {d0, d1, d2, d3},   [r1:128]! \n\t" // load C0
		"vadd.f32  q6, q2, q6            \n\t"
		"vadd.f32  q7, q3, q7            \n\t"
		"vld1.64   {d4, d5, d6, d7},   [r1:128]  \n\t" // load C0
		"vadd.f32  q8, q0, q8            \n\t"
		"vadd.f32  q9, q1, q9            \n\t"
		"vld1.64   {d0, d1, d2, d3},   [r2:128]! \n\t" // load C0
		"vadd.f32  q10, q2, q10            \n\t"
		"vadd.f32  q11, q3, q11            \n\t"
		"vld1.64   {d4, d5, d6, d7},   [r2:128]  \n\t" // load C0
		"vadd.f32  q12, q0, q12            \n\t"
		"vadd.f32  q13, q1, q13            \n\t"
		"vadd.f32  q14, q2, q14            \n\t"
		"vadd.f32  q15, q3, q15            \n\t"
		"                                \n\t"
		".D0_12x4:                            \n\t" // alg==0
		"                                \n\t"
		"ldr    r0, %9                   \n\t" // load address of C0
		"ldr    r1, %10                  \n\t" // load address of C1
		"ldr    r2, %11                  \n\t" // load address of C2
		"                                \n\t"
		"vst1.64   {d8, d9, d10, d11},     [r0:128]!  \n\t" // store D0
		"vst1.64   {d12, d13, d14, d15},   [r0:128]  \n\t" // store D0
		"vst1.64   {d16, d17, d18, d19},   [r1:128]!  \n\t" // store D0
		"vst1.64   {d20, d21, d22, d23},   [r1:128]  \n\t" // store D0
		"vst1.64   {d24, d25, d26, d27},   [r2:128]!  \n\t" // store D0
		"vst1.64   {d28, d29, d30, d31},   [r2:128]  \n\t" // store D0
/*		"                                \n\t"*/
/*		"                                \n\t"*/
		"                                \n\t"
		".align 3                        \n\t"
		".DZERO_12x4:                    \n\t" // zero quad word
		".word  0                        \n\t"
		".word  0                        \n\t"
		".word  0                        \n\t"
		".word  0                        \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		: // output operands (none)
		: // input operands
		  "r" (k_iter),		// %0
		  "r" (k_left),		// %1
		  "r" (A0),			// %2
		  "r" (A1),			// %3
		  "r" (A2),			// %4
		  "r" (B),			// %5
		  "m" (C0),			// %6
		  "m" (C1),			// %7
		  "m" (C2),			// %8
		  "m" (D0),			// %9
		  "m" (D1),			// %10
		  "m" (D2),			// %11
		  "r" (alg)			// %12
		: // register clobber list
		  "r0", "r1", "r2",
		  "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",
		  "d8", "d9", "d10", "d11", "d12", "d13", "d14", "d15",
		  "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23",
		  "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31",
		  "memory"
	);
}



// normal-transposed, 8x4 with data packed in 4
void kernel_sgemm_nt_8x4_lib4(int kmax, float *A0, float *A1, float *B, float *C0, float *C1, float *D0, float *D1, int alg)
	{
	
	if(kmax<=0)
		return;
		
	__builtin_prefetch( A0 );
	__builtin_prefetch( A1 );
	__builtin_prefetch( B  );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A0+8 );
	__builtin_prefetch( A1+8 );
	__builtin_prefetch( B +8 );
#endif

	int k_iter = kmax/4;
	int k_left = kmax%4;
	

	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"pld    [%2, #64]                \n\t" // prefetch A0 to L1
		"pld    [%3, #64]                \n\t" // prefetch A1 to L1
		"pld    [%4, #64]                \n\t" // prefetch B to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%2, #96]                \n\t" // prefetch A0 to L1
		"pld    [%3, #96]                \n\t" // prefetch A1 to L1
		"pld    [%4, #96]                \n\t" // prefetch B to L1
#endif
		"                                \n\t"
		"                                \n\t"
		"mov    r0, %0                   \n\t" // k_iter
		"                                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)
		"vld1.64   {d12, d13, d14, d15}, [%4:128]! \n\t" // load B to registers
		"vld1.64   {d4, d5, d6, d7},     [%4:128] \n\t" // load B to registers
		"vld1.64   {d8, d9, d10, d11},   [%2:128] \n\t" // load A0 to registers
		"vld1.64   {d24, d25},           [%3:128] \n\t" // load A1
#endif
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)
		"vld1.64   {d12, d13, d14, d15}, [%4:128]! \n\t" // load B to registers
		"vld1.64   {d8, d9, d10, d11},   [%2:128]! \n\t" // load A0 to registers
		"vld1.64   {d24, d25, d26, d27}, [%3:128]! \n\t" // load A1
#endif
		"                                \n\t"
		"cmp    r0, #0                   \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vldr   d0, .DZERO_8x4               \n\t" // load zero double
		"vldr   d1, .DZERO_8x4+8             \n\t" // load zero double
		"vmov   q1, q0                   \n\t"
/*		"vmov   q2, q0                   \n\t"*/
/*		"vmov   q3, q0                   \n\t"*/
		"vmov   q14, q0                   \n\t"
		"vmov   q15, q0                   \n\t"
		"vmov   q8, q0                   \n\t"
		"vmov   q9, q0                   \n\t"
		"vmov   q10, q0                  \n\t"
		"vmov   q11, q0                  \n\t"
		"                                \n\t"
		"                                \n\t"
		"ble    .DCONSIDERLEFT2_8x4           \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DLOOPKITER_8x4:                    \n\t" // main loop
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)
		"                                \n\t"
		"vmla.f32  q0, q4, d12[0]        \n\t"
		"vldr   d26, [%3, #16]             \n\t"
		"vmla.f32  q1, q4, d12[1]        \n\t"
		"vldr   d27, [%3, #24]             \n\t"
		"vmla.f32  q14, q4, d13[0]        \n\t"
		"pld    [%2, #128]                \n\t"
		"vmla.f32  q15, q4, d13[1]        \n\t"
		"vldr   d8, [%2, #32]             \n\t"
		"                                \n\t"
		"vmla.f32  q8, q12, d12[0]        \n\t"
		"vldr   d9, [%2, #40]             \n\t"
		"vmla.f32  q9, q12, d12[1]        \n\t"
		"pld    [%3, #128]                \n\t"
		"vmla.f32  q10, q12, d13[0]        \n\t"
		"pld    [%4, #128]                \n\t"
		"vmla.f32  q11, q12, d13[1]        \n\t"
		"vldr   d24, [%3, #32]             \n\t"
		"                                \n\t"
		"vmla.f32  q0, q5, d14[0]        \n\t"
		"vldr   d25, [%3, #40]             \n\t"
		"vmla.f32  q1, q5, d14[1]        \n\t"
		"vldr   d12, [%4, #32]             \n\t"
		"vmla.f32  q14, q5, d15[0]        \n\t"
		"vldr   d13, [%4, #40]             \n\t"
		"vmla.f32  q15, q5, d15[1]        \n\t"
		"vldr   d10, [%2, #48]             \n\t"
		"                                \n\t"
		"vmla.f32  q8, q13, d14[0]        \n\t"
		"vldr   d11, [%2, #56]             \n\t"
		"vmla.f32  q9, q13, d14[1]        \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"vmla.f32  q10, q13, d15[0]        \n\t"
		"vldr   d14, [%4, #48]             \n\t"
		"vmla.f32  q11, q13, d15[1]        \n\t"
		"vldr   d26, [%3, #48]             \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q0, q4, d4[0]        \n\t"
		"vldr   d27, [%3, #56]             \n\t"
		"vmla.f32  q1, q4, d4[1]        \n\t"
		"vldr   d15, [%4, #56]             \n\t"
		"vmla.f32  q14, q4, d5[0]        \n\t"
		"cmp    r0, #0                   \n\t" // next iter?
		"vmla.f32  q15, q4, d5[1]        \n\t"
		"vldr   d8, [%2, #64]             \n\t"
		"                                \n\t"
		"vmla.f32  q8, q12, d4[0]        \n\t"
		"vldr   d9, [%2, #72]             \n\t"
		"vmla.f32  q9, q12, d4[1]        \n\t"
		"vmla.f32  q10, q12, d5[0]        \n\t"
		"vldr   d4, [%4, #64]             \n\t"
		"vmla.f32  q11, q12, d5[1]        \n\t"
		"vldr   d24, [%3, #64]             \n\t"
		"                                \n\t"
		"vmla.f32  q0, q5, d6[0]        \n\t"
		"vldr   d25, [%3, #72]             \n\t"
		"vmla.f32  q1, q5, d6[1]        \n\t"
		"vldr   d5, [%4, #72]             \n\t"
		"vmla.f32  q14, q5, d7[0]        \n\t"
		"add    %3, %3, #64              \n\t" // increase A
		"vmla.f32  q15, q5, d7[1]        \n\t"
		"vldr   d10, [%2, #80]             \n\t"
		"                                \n\t"
		"vmla.f32  q8, q13, d6[0]        \n\t"
		"vldr   d11, [%2, #88]             \n\t"
		"vmla.f32  q9, q13, d6[1]        \n\t"
		"add    %2, %2, #64              \n\t" // increase A
		"vmla.f32  q10, q13, d7[0]        \n\t"
		"vldr   d6, [%4, #80]             \n\t"
		"vmla.f32  q11, q13, d7[1]        \n\t"
		"vldr   d7, [%4, #88]             \n\t"
		"add    %4, %4, #64              \n\t" // increase A
		"                                \n\t"
#endif
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)
		"                                \n\t"
		"pld    [%2, #96]                \n\t"
		"pld    [%3, #96]                \n\t"
		"pld    [%4, #96]                \n\t"
		"                                \n\t"
		"vmla.f32  q0, q4, d12[0]        \n\t"
		"vmla.f32  q1, q4, d12[1]        \n\t"
		"vmla.f32  q14, q4, d13[0]        \n\t"
		"vmla.f32  q15, q4, d13[1]        \n\t"
		"                                \n\t"
		"vmla.f32  q8, q12, d12[0]        \n\t"
		"vmla.f32  q9, q12, d12[1]        \n\t"
		"vmla.f32  q10, q12, d13[0]        \n\t"
		"vmla.f32  q11, q12, d13[1]        \n\t"
		"                                \n\t"
		"vmla.f32  q0, q5, d14[0]        \n\t"
		"vmla.f32  q1, q5, d14[1]        \n\t"
		"vmla.f32  q14, q5, d15[0]        \n\t"
		"vmla.f32  q15, q5, d15[1]        \n\t"
		"                                \n\t"
		"vmla.f32  q8, q13, d14[0]        \n\t"
		"vmla.f32  q9, q13, d14[1]        \n\t"
		"vmla.f32  q10, q13, d15[0]        \n\t"
		"vmla.f32  q11, q13, d15[1]        \n\t"
		"                                \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"                                \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%4:128]! \n\t" // load B
		"vld1.64   {d8, d9, d10, d11},   [%2:128]! \n\t" // load A0
		"vld1.64   {d24, d25, d26, d27}, [%3:128]! \n\t" // load A0
		"                                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A9)
		"pld    [%2, #96]                \n\t"
		"pld    [%3, #96]                \n\t"
		"pld    [%4, #96]                \n\t"
#endif
		"                                \n\t"
		"vmla.f32  q0, q4, d12[0]        \n\t"
		"vmla.f32  q1, q4, d12[1]        \n\t"
		"vmla.f32  q14, q4, d13[0]        \n\t"
		"vmla.f32  q15, q4, d13[1]        \n\t"
		"                                \n\t"
		"vmla.f32  q8, q12, d12[0]        \n\t"
		"vmla.f32  q9, q12, d12[1]        \n\t"
		"vmla.f32  q10, q12, d13[0]        \n\t"
		"vmla.f32  q11, q12, d13[1]        \n\t"
		"                                \n\t"
		"vmla.f32  q0, q5, d14[0]        \n\t"
		"vmla.f32  q1, q5, d14[1]        \n\t"
		"vmla.f32  q14, q5, d15[0]        \n\t"
		"vmla.f32  q15, q5, d15[1]        \n\t"
		"                                \n\t"
		"vmla.f32  q8, q13, d14[0]        \n\t"
		"vmla.f32  q9, q13, d14[1]        \n\t"
		"vmla.f32  q10, q13, d15[0]        \n\t"
		"vmla.f32  q11, q13, d15[1]        \n\t"
		"                                \n\t"
		"cmp    r0, #0                   \n\t" // next iter?
		"                                \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%4:128]! \n\t" // load B
		"vld1.64   {d8, d9, d10, d11},   [%2:128]! \n\t" // load A0
		"vld1.64   {d24, d25, d26, d27}, [%3:128]! \n\t" // load A0
		"                                \n\t"
#endif
		"                                \n\t"
		"bgt    .DLOOPKITER_8x4              \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERLEFT2_8x4:                 \n\t" // consider left k+=2
		"                                \n\t"
		"mov    r0, %1                   \n\t" // k_left
		"cmp    r0, #1                   \n\t"
		"ble    .DCONSIDERLEFT1_8x4          \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)
		"                                \n\t"
		"vmla.f32  q0, q4, d12[0]        \n\t"
		"vldr   d26, [%3, #16]             \n\t"
		"vmla.f32  q1, q4, d12[1]        \n\t"
		"vldr   d27, [%3, #24]             \n\t"
		"vmla.f32  q14, q4, d13[0]        \n\t"
		"vmla.f32  q15, q4, d13[1]        \n\t"
		"vldr   d8, [%2, #32]             \n\t"
		"                                \n\t"
		"vmla.f32  q8, q12, d12[0]        \n\t"
		"vldr   d9, [%2, #40]             \n\t"
		"vmla.f32  q9, q12, d12[1]        \n\t"
		"vmla.f32  q10, q12, d13[0]        \n\t"
		"vmla.f32  q11, q12, d13[1]        \n\t"
		"vldr   d24, [%3, #32]             \n\t"
		"                                \n\t"
		"vmla.f32  q0, q5, d14[0]        \n\t"
		"vldr   d25, [%3, #40]             \n\t"
		"vmla.f32  q1, q5, d14[1]        \n\t"
/*		"vldr   d12, [%4, #32]             \n\t"*/
		"vmov   d12, d4                  \n\t"
		"vmla.f32  q14, q5, d15[0]        \n\t"
/*		"vldr   d13, [%4, #40]             \n\t"*/
		"vmov   d13, d5                  \n\t"
		"vmla.f32  q15, q5, d15[1]        \n\t"
		"                                \n\t"
		"vmla.f32  q8, q13, d14[0]        \n\t"
		"vmla.f32  q9, q13, d14[1]        \n\t"
		"vmla.f32  q10, q13, d15[0]        \n\t"
		"vmla.f32  q11, q13, d15[1]        \n\t"
		"                                \n\t"
#endif
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)
		"                                \n\t"
		"vmla.f32  q0, q4, d12[0]        \n\t"
		"vmla.f32  q1, q4, d12[1]        \n\t"
		"vmla.f32  q14, q4, d13[0]        \n\t"
		"vmla.f32  q15, q4, d13[1]        \n\t"
		"                                \n\t"
		"vmla.f32  q8, q12, d12[0]        \n\t"
		"vmla.f32  q9, q12, d12[1]        \n\t"
		"vmla.f32  q10, q12, d13[0]        \n\t"
		"vmla.f32  q11, q12, d13[1]        \n\t"
		"                                \n\t"
		"vmla.f32  q0, q5, d14[0]        \n\t"
		"vmla.f32  q1, q5, d14[1]        \n\t"
		"vmla.f32  q14, q5, d15[0]        \n\t"
		"vmla.f32  q15, q5, d15[1]        \n\t"
		"                                \n\t"
		"vmla.f32  q8, q13, d14[0]        \n\t"
		"vmla.f32  q9, q13, d14[1]        \n\t"
		"vmla.f32  q10, q13, d15[0]        \n\t"
		"vmla.f32  q11, q13, d15[1]        \n\t"
		"                                \n\t"
		"vld1.64   {d12, d13}, [%4:128] \n\t" // load B
		"vld1.64   {d8, d9},   [%2:128] \n\t" // load A0
		"vld1.64   {d24, d25}, [%3:128] \n\t" // load A1
		"                                \n\t"
#endif
		"                                \n\t"
		"sub    r0, r0, #2               \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERLEFT1_8x4:                 \n\t" // consider left k++
		"                                \n\t"
		"cmp    r0, #0                   \n\t"
		"ble    .DPOSTACCUM_8x4              \n\t"
		"                                \n\t"
		"vmla.f32  q0, q4, d12[0]        \n\t"
		"vmla.f32  q1, q4, d12[1]        \n\t"
		"vmla.f32  q14, q4, d13[0]        \n\t"
		"vmla.f32  q15, q4, d13[1]        \n\t"
		"                                \n\t"
		"vmla.f32  q8, q12, d12[0]        \n\t"
		"vmla.f32  q9, q12, d12[1]        \n\t"
		"vmla.f32  q10, q12, d13[0]        \n\t"
		"vmla.f32  q11, q12, d13[1]        \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DPOSTACCUM_8x4:                    \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmov      q2, q14               \n\t"
		"vmov      q3, q15               \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"cmp    %7, #0                   \n\t"
		"beq    .D0_8x4                      \n\t" // if alg==0, jump
		"                                \n\t"
		"cmp    %7, #1                   \n\t"
		"                                \n\t"
		"vld1.64   {d8, d9, d10, d11},   [%5:128]! \n\t" // load C0
		"vld1.64   {d12, d13, d14, d15}, [%5:128]  \n\t" // load C0
		"vld1.64   {d24, d25, d26, d27}, [%6:128]! \n\t" // load C1
		"vld1.64   {d28, d29, d30, d31}, [%6:128]  \n\t" // load C1
		"                                \n\t"
		"                                \n\t"
		"beq    .D1_8x4                      \n\t" // if alg==1, jump
		"                                \n\t"
		"                                \n\t"// alg==-1
		"vsub.f32  q0, q4, q0            \n\t"
		"vsub.f32  q1, q5, q1            \n\t"
		"vsub.f32  q2, q6, q2            \n\t"
		"vsub.f32  q3, q7, q3            \n\t"
		"                                \n\t"
		"vsub.f32  q8,  q12, q8            \n\t"
		"vsub.f32  q9,  q13, q9            \n\t"
		"vsub.f32  q10, q14, q10           \n\t"
		"vsub.f32  q11, q15, q11           \n\t"
		"                                \n\t"
		"b      .D0_8x4                      \n\t" // jump to end
		"                                \n\t"
		"                                \n\t"
		".D1_8x4:                            \n\t" // alg==1
		"                                \n\t"
		"vadd.f32  q0, q0, q4            \n\t"
		"vadd.f32  q1, q1, q5            \n\t"
		"vadd.f32  q2, q2, q6            \n\t"
		"vadd.f32  q3, q3, q7            \n\t"
		"                                \n\t"
		"vadd.f32  q8,  q8,  q12            \n\t"
		"vadd.f32  q9,  q9,  q13           \n\t"
		"vadd.f32  q10, q10, q14           \n\t"
		"vadd.f32  q11, q11, q15           \n\t"
		"                                \n\t"
		".D0_8x4:                            \n\t" // alg==0
		"                                \n\t"
		"vst1.64   {d0, d1, d2, d3},     [%8:128]!  \n\t" // store C
		"vst1.64   {d4, d5, d6, d7},     [%8:128]   \n\t" // store C
		"vst1.64   {d16, d17, d18, d19}, [%9:128]!  \n\t" // store C
		"vst1.64   {d20, d21, d22, d23}, [%9:128]   \n\t" // store C
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".align 3                        \n\t"
		".DZERO_8x4:                         \n\t" // zero quad word
		".word  0                        \n\t"
		".word  0                        \n\t"
		".word  0                        \n\t"
		".word  0                        \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		: // output operands (none)
		: // input operands
		  "r" (k_iter),		// %0
		  "r" (k_left),		// %1
		  "r" (A0),			// %2
		  "r" (A1),			// %3
		  "r" (B),			// %4
		  "r" (C0),			// %5
		  "r" (C1),			// %6
		  "r" (alg),		// %7
		  "r" (D0),			// %8
		  "r" (D1)			// %9
		: // register clobber list
		  "r0",
		  "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",
		  "d8", "d9", "d10", "d11", "d12", "d13", "d14", "d15",
		  "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23",
		  "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31",
		  "memory"
	);
}



// normal-transposed, 4x4 with data packed in 4
void kernel_sgemm_nt_4x4_lib4(int kmax, float *A, float *B, float *C, float *D, int alg)
	{
	
	if(kmax<=0)
		return;
		
	__builtin_prefetch( A );
	__builtin_prefetch( B );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A+8 );
	__builtin_prefetch( B+8 );
#endif

	int k_iter = kmax/4;
	int k_left = kmax%4;
	
//	printf("\n%d %d %d\n", kmax, k_iter, k_left);

	__asm__ volatile
	(
		"                                \n\t"
/*		"mov    %2, %2                   \n\t" // load address of A*/
/*		"mov    %3, %3                   \n\t" // load address of B*/
		"                                \n\t"
		"                                \n\t"
		"pld    [%2, #64]                \n\t" // prefetch A to L1
		"pld    [%3, #64]                \n\t" // prefetch B to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%2, #96]                \n\t" // prefetch A to L1
		"pld    [%3, #96]                \n\t" // prefetch B to L1
#endif
//		"pld    [%2, #128]                \n\t"
//		"pld    [%3, #128]                \n\t"
		"                                \n\t"
		"                                \n\t"
//		"ldr    r0, %0                   \n\t" // k_iter
		"mov    r0, %0                   \n\t" // k_iter
		"                                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)
		"vld1.64   {d12, d13, d14, d15}, [%3:128] \n\t" // load B to registers
		"vld1.64   {d8, d9}, [%2:128]   \n\t" // load A to registers
#endif
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)
		"vld1.64   {d12, d13, d14, d15}, [%3:128]! \n\t" // load B to registers
		"vld1.64   {d8, d9, d10, d11},   [%2:128]! \n\t" // load A0 to registers
#endif
		"                                \n\t"
		"cmp    r0, #0                   \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vldr   d0, .DZERO_4x4               \n\t" // load zero double
		"vldr   d1, .DZERO_4x4+8             \n\t" // load zero double
		"vmov   q1, q0                   \n\t"
		"vmov   q2, q0                   \n\t"
		"vmov   q3, q0                   \n\t"
		"                                \n\t"
		"                                \n\t"
		"ble    .DCONSIDERLEFT2_4x4           \n\t"
		"                                \n\t"
		"                                \n\t"
		".DLOOPKITER_4x4:                    \n\t" // main loop
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)
		"                                \n\t"
		"pld    [%2, #128]                \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"pld    [%3, #128]                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q0, q4, d12[0]        \n\t"
		"vldr   d10, [%2, #16]             \n\t"
		"vmla.f32  q1, q4, d12[1]        \n\t"
		"vldr   d12, [%3, #32]             \n\t"
		"vmla.f32  q2, q4, d13[0]        \n\t"
		"vldr   d11, [%2, #24]             \n\t"
		"vmla.f32  q3, q4, d13[1]        \n\t"
		"vldr   d13, [%3, #40]             \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q0, q5, d14[0]        \n\t"
		"vldr   d8, [%2, #32]             \n\t"
		"vmla.f32  q1, q5, d14[1]        \n\t"
		"vldr   d14, [%3, #48]             \n\t"
		"vmla.f32  q2, q5, d15[0]        \n\t"
		"vldr   d9, [%2, #40]             \n\t"
		"vmla.f32  q3, q5, d15[1]        \n\t"
		"vldr   d15, [%3, #56]             \n\t"
		"                                \n\t"
/*		"vld1.64   {d12, d13, d14, d15}, [%3:128]! \n\t" // load B*/
/*		"vld1.64   {d8, d9, d10, d11}, [%2:128]!   \n\t" // load A*/
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"cmp    r0, #0                   \n\t" // next iter?
		"                                \n\t"
		"vmla.f32  q0, q4, d12[0]        \n\t"
		"vldr   d10, [%2, #48]             \n\t"
		"vmla.f32  q1, q4, d12[1]        \n\t"
		"vldr   d12, [%3, #64]             \n\t"
		"vmla.f32  q2, q4, d13[0]        \n\t"
		"vldr   d11, [%2, #56]             \n\t"
		"vmla.f32  q3, q4, d13[1]        \n\t"
		"vldr   d13, [%3, #72]             \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q0, q5, d14[0]        \n\t"
		"vldr   d8, [%2, #64]             \n\t"
		"vmla.f32  q1, q5, d14[1]        \n\t"
		"vldr   d14, [%3, #80]             \n\t"
		"vmla.f32  q2, q5, d15[0]        \n\t"
		"vldr   d9, [%2, #72]             \n\t"
		"vmla.f32  q3, q5, d15[1]        \n\t"
		"vldr   d15, [%3, #88]             \n\t"
		"                                \n\t"
		"add    %2, %2, #64              \n\t" // increase A
		"add    %3, %3, #64              \n\t" // increase A
		"                                \n\t"
		"                                \n\t"
/*		"vld1.64   {d12, d13, d14, d15}, [%3:128]! \n\t" // load B*/
/*		"vld1.64   {d8, d9, d10, d11}, [%2:128]!   \n\t" // load A*/
#endif
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)
		"                                \n\t"
		"pld    [%2, #96]                \n\t"
		"pld    [%3, #96]                \n\t"
		"                                \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"                                \n\t"
		"vmla.f32  q0, q4, d12[0]        \n\t"
		"vmla.f32  q1, q4, d12[1]        \n\t"
		"vmla.f32  q2, q4, d13[0]        \n\t"
		"vmla.f32  q3, q4, d13[1]        \n\t"
		"                                \n\t"
		"vmla.f32  q0, q5, d14[0]        \n\t"
		"vmla.f32  q1, q5, d14[1]        \n\t"
		"vmla.f32  q2, q5, d15[0]        \n\t"
		"vmla.f32  q3, q5, d15[1]        \n\t"
		"                                \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%3:128]! \n\t" // load B
		"vld1.64   {d8, d9, d10, d11},   [%2:128]! \n\t" // load A0
		"                                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A9)
		"pld    [%2, #96]                \n\t"
		"pld    [%3, #96]                \n\t"
#endif
		"                                \n\t"
		"cmp    r0, #0                   \n\t" // next iter?
		"                                \n\t"
		"vmla.f32  q0, q4, d12[0]        \n\t"
		"vmla.f32  q1, q4, d12[1]        \n\t"
		"vmla.f32  q2, q4, d13[0]        \n\t"
		"vmla.f32  q3, q4, d13[1]        \n\t"
		"                                \n\t"
		"vmla.f32  q0, q5, d14[0]        \n\t"
		"vmla.f32  q1, q5, d14[1]        \n\t"
		"vmla.f32  q2, q5, d15[0]        \n\t"
		"vmla.f32  q3, q5, d15[1]        \n\t"
		"                                \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%3:128]! \n\t" // load B
		"vld1.64   {d8, d9, d10, d11},   [%2:128]! \n\t" // load A0
		"                                \n\t"
#endif
		"                                \n\t"
		"bgt    .DLOOPKITER_4x4              \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERLEFT2_4x4:                 \n\t" // consider left k+=2
		"                                \n\t"
		"mov    r0, %1                   \n\t" // k_left
		"cmp    r0, #1                   \n\t"
		"ble    .DCONSIDERLEFT1_4x4          \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)
		"vmla.f32  q0, q4, d12[0]        \n\t"
		"vldr   d10, [%2, #16]             \n\t"
		"vmla.f32  q1, q4, d12[1]        \n\t"
		"vldr   d12, [%3, #32]             \n\t"
		"vmla.f32  q2, q4, d13[0]        \n\t"
		"vldr   d11, [%2, #24]             \n\t"
		"vmla.f32  q3, q4, d13[1]        \n\t"
		"vldr   d13, [%3, #40]             \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q0, q5, d14[0]        \n\t"
		"vldr   d8, [%2, #32]             \n\t"
		"vmla.f32  q1, q5, d14[1]        \n\t"
/*		"vldr   d14, [%3, #48]             \n\t"*/
		"vmla.f32  q2, q5, d15[0]        \n\t"
		"vldr   d9, [%2, #40]             \n\t"
		"vmla.f32  q3, q5, d15[1]        \n\t"
/*		"vldr   d15, [%3, #56]             \n\t"*/
#endif
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)
		"vmla.f32  q0, q4, d12[0]        \n\t"
		"vmla.f32  q1, q4, d12[1]        \n\t"
		"vmla.f32  q2, q4, d13[0]        \n\t"
		"vmla.f32  q3, q4, d13[1]        \n\t"
		"                                \n\t"
		"vmla.f32  q0, q5, d14[0]        \n\t"
		"vmla.f32  q1, q5, d14[1]        \n\t"
		"vmla.f32  q2, q5, d15[0]        \n\t"
		"vmla.f32  q3, q5, d15[1]        \n\t"
		"                                \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%3:128]! \n\t" // load B
		"vld1.64   {d8, d9, d10, d11}, [%2:128]!   \n\t" // load A
#endif
		"                                \n\t"
		"sub    r0, r0, #2               \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERLEFT1_4x4:                 \n\t" // consider left k++
		"                                \n\t"
		"cmp    r0, #0                   \n\t"
		"ble    .DPOSTACCUM_4x4              \n\t"
		"                                \n\t"
		"vmla.f32  q0, q4, d12[0]        \n\t"
		"vmla.f32  q1, q4, d12[1]        \n\t"
		"vmla.f32  q2, q4, d13[0]        \n\t"
		"vmla.f32  q3, q4, d13[1]        \n\t"
		"                                \n\t"
/*		"vld1.64   {d12, d13}, [%3:128]  \n\t" // no need to increment pointer*/
/*		"vld1.64   {d8, d9}, [%2:128]    \n\t" // no need to increment pointer*/
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DPOSTACCUM_4x4:                    \n\t"
		"                                \n\t"
/*		"mov    %3, %4                   \n\t" // load address of C*/
		"                                \n\t"
		"mov    r0, %5                   \n\t" // alg
		"cmp    r0, #0                   \n\t"
		"beq    .D0_4x4                      \n\t" // if alg==0, jump
		"                                \n\t"
		"cmp    r0, #1                   \n\t"
		"                                \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%4:128]!  \n\t" // load C
		"vld1.64   {d12, d13, d14, d15}, [%4:128] \n\t" // load C
		"                                \n\t"
/*		"mov    %3, %6                   \n\t" // load address of D*/
		"                                \n\t"
		"beq    .D1_4x4                      \n\t" // if alg==1, jump
		"                                \n\t"
		"                                \n\t"// alg==-1
		"vsub.f32  q0, q4, q0            \n\t"
		"vsub.f32  q1, q5, q1            \n\t"
		"vsub.f32  q2, q6, q2            \n\t"
		"vsub.f32  q3, q7, q3            \n\t"
		"                                \n\t"
		"b      .D0_4x4                      \n\t" // jump to end
		"                                \n\t"
		"                                \n\t"
		".D1_4x4:                            \n\t" // alg==1
		"                                \n\t"
		"vadd.f32  q0, q0, q4            \n\t"
		"vadd.f32  q1, q1, q5            \n\t"
		"vadd.f32  q2, q2, q6            \n\t"
		"vadd.f32  q3, q3, q7            \n\t"
		"                                \n\t"
		".D0_4x4:                            \n\t" // alg==0
		"                                \n\t"
		"vst1.64   {d0, d1, d2, d3}, [%6:128]!  \n\t" // store D
		"vst1.64   {d4, d5, d6, d7}, [%6:128]   \n\t" // store D
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".align 3                        \n\t"
		".DZERO_4x4:                         \n\t" // zero quad word
		".word  0                        \n\t"
		".word  0                        \n\t"
		".word  0                        \n\t"
		".word  0                        \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		: // output operands (none)
		: // input operands
		  "r" (k_iter),		// %0
		  "r" (k_left),		// %1
		  "r" (A),			// %2
		  "r" (B),			// %3
		  "r" (C),			// %4
		  "r" (alg),		// %5
		  "r" (D)			// %6
		: // register clobber list
		  "r0",
		  "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7",
		  "s8", "s9", "s10", "s11", "s12", "s13", "s14", "s15",
		  "s16", "s17", "s18", "s19", "s20", "s21", "s22", "s23",
		  "s24", "s25", "s26", "s27", "s28", "s29", "s30", "s31",
		  "memory"
	);
}



// normal-transposed, 4x2 with data packed in 4
void kernel_sgemm_nt_4x2_lib4(int kmax, float *A, float *B, float *D, float *C, int alg)
	{
	
	if(kmax<=0)
		return;
	
	const int lda = 4;
	const int ldc = 4;

	int k;

	float
		a_0, a_1, a_2, a_3,
		b_0, b_1,
		c_00=0, c_01=0,
		c_10=0, c_11=0,
		c_20=0, c_21=0,
		c_30=0, c_31=0;
		
	for(k=0; k<kmax-3; k+=4)
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
	for(; k<kmax; k++)
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


		A += 4;
		B += 4;

		}
		
	float
		d_00, d_01,
		d_10, d_11,
		d_20, d_21,
		d_30, d_31;
	
	if(alg==0) // C = A * B'
		{
		C[0+ldc*0] = c_00;
		C[1+ldc*0] = c_10;
		C[2+ldc*0] = c_20;
		C[3+ldc*0] = c_30;

		C[0+ldc*1] = c_01;
		C[1+ldc*1] = c_11;
		C[2+ldc*1] = c_21;
		C[3+ldc*1] = c_31;
		}
	else 
		{
		d_00 = D[0+ldc*0];
		d_10 = D[1+ldc*0];
		d_20 = D[2+ldc*0];
		d_30 = D[3+ldc*0];
		
		d_01 = D[0+ldc*1];
		d_11 = D[1+ldc*1];
		d_21 = D[2+ldc*1];
		d_31 = D[3+ldc*1];
		
		if(alg==1) // C += A * B'
			{
			d_00 += c_00;
			d_10 += c_10;
			d_20 += c_20;
			d_30 += c_30;

			d_01 += c_01;
			d_11 += c_11;
			d_21 += c_21;
			d_31 += c_31;
			}
		else // C -= A * B'
			{
			d_00 -= c_00;
			d_10 -= c_10;
			d_20 -= c_20;
			d_30 -= c_30;

			d_01 -= c_01;
			d_11 -= c_11;
			d_21 -= c_21;
			d_31 -= c_31;
			}

		C[0+ldc*0] = d_00;
		C[1+ldc*0] = d_10;
		C[2+ldc*0] = d_20;
		C[3+ldc*0] = d_30;

		C[0+ldc*1] = d_01;
		C[1+ldc*1] = d_11;
		C[2+ldc*1] = d_21;
		C[3+ldc*1] = d_31;
		}
	
	}



// normal-transposed, 2x4 with data packed in 4
void kernel_sgemm_nt_2x4_lib4(int kmax, float *A, float *B, float *D, float *C, int alg)
	{
	
	if(kmax<=0)
		return;
	
	const int lda = 4;
	const int ldc = 4;

	int k;

	float
		a_0, a_1,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0, c_02=0, c_03=0,
		c_10=0, c_11=0, c_12=0, c_13=0;
		
	for(k=0; k<kmax-3; k+=4)
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
	for(; k<kmax; k++)
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


		A += 4;
		B += 4;

		}
		
	float
		d_00, d_01, d_02, d_03,
		d_10, d_11, d_12, d_13;
	
	if(alg==0) // C = A * B'
		{
		C[0+ldc*0] = c_00;
		C[1+ldc*0] = c_10;

		C[0+ldc*1] = c_01;
		C[1+ldc*1] = c_11;

		C[0+ldc*2] = c_02;
		C[1+ldc*2] = c_12;

		C[0+ldc*3] = c_03;
		C[1+ldc*3] = c_13;
		}
	else 
		{
		d_00 = D[0+ldc*0];
		d_10 = D[1+ldc*0];
		
		d_01 = D[0+ldc*1];
		d_11 = D[1+ldc*1];
		
		d_02 = D[0+ldc*2];
		d_12 = D[1+ldc*2];
		
		d_03 = D[0+ldc*3];
		d_13 = D[1+ldc*3];
		
		if(alg==1) // C += A * B'
			{
			d_00 += c_00;
			d_10 += c_10;

			d_01 += c_01;
			d_11 += c_11;

			d_02 += c_02;
			d_12 += c_12;

			d_03 += c_03;
			d_13 += c_13;
			}
		else // C -= A * B'
			{
			d_00 -= c_00;
			d_10 -= c_10;

			d_01 -= c_01;
			d_11 -= c_11;

			d_02 -= c_02;
			d_12 -= c_12;

			d_03 -= c_03;
			d_13 -= c_13;
			}

		C[0+ldc*0] = d_00;
		C[1+ldc*0] = d_10;

		C[0+ldc*1] = d_01;
		C[1+ldc*1] = d_11;

		C[0+ldc*2] = d_02;
		C[1+ldc*2] = d_12;

		C[0+ldc*3] = d_03;
		C[1+ldc*3] = d_13;
		}
	
	}



// normal-transposed, 2x2 with data packed in 4
void kernel_sgemm_nt_2x2_lib4(int kmax, float *A, float *B, float *D, float *C, int alg)
	{
	
	if(kmax<=0)
		return;
	
	const int lda = 4;
	const int ldc = 4;

	int k;

	float
		a_0, a_1,
		b_0, b_1,
		c_00=0, c_01=0,
		c_10=0, c_11=0;
		
	for(k=0; k<kmax-3; k+=4)
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
	for(; k<kmax; k++)
		{
		
		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		
		b_0 = B[0+lda*0];
		b_1 = B[1+lda*0];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_01 += a_0 * b_1;
		c_11 += a_1 * b_1;


		A += 4;
		B += 4;

		}
		
	float
		d_00, d_01,
		d_10, d_11;
	
	if(alg==0) // C = A * B'
		{
		C[0+ldc*0] = c_00;
		C[1+ldc*0] = c_10;

		C[0+ldc*1] = c_01;
		C[1+ldc*1] = c_11;
		}
	else 
		{
		d_00 = D[0+ldc*0];
		d_10 = D[1+ldc*0];
		
		d_01 = D[0+ldc*1];
		d_11 = D[1+ldc*1];
		
		if(alg==1) // C += A * B'
			{
			d_00 += c_00;
			d_10 += c_10;

			d_01 += c_01;
			d_11 += c_11;
			}
		else // C -= A * B'
			{
			d_00 -= c_00;
			d_10 -= c_10;

			d_01 -= c_01;
			d_11 -= c_11;
			}

		C[0+ldc*0] = d_00;
		C[1+ldc*0] = d_10;

		C[0+ldc*1] = d_01;
		C[1+ldc*1] = d_11;
		}
	
	}

