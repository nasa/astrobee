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



void kernel_strmm_nt_12x4_lib4(int kmax, float *A0, float *A1, float *A2, float *B, float *C0, float *C1, float *C2)
	{
	
/*	if(kmax<=0)*/
/*		return;*/
		
	__builtin_prefetch( A0 );
	__builtin_prefetch( A1 );
	__builtin_prefetch( A2 );
	__builtin_prefetch( B  );

	int k_iter = kmax/4 - 1;
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
		"                                \n\t"
		"                                \n\t"
/*		"ldr    r4, %5                   \n\t" // load address of C*/
/*		"ldr    r5, %6                   \n\t" // load address of C*/
/*		"ldr    r6, %7                   \n\t" // alg*/
		"                                \n\t"
		"                                \n\t"
/*		"vldr   d8, .DZERO12               \n\t" // load zero double*/
/*		"vldr   d9, .DZERO12+8             \n\t" // load zero double*/
/*		"vmov   q5, q4                   \n\t"*/
/*		"vmov   q6, q4                   \n\t"*/
/*		"vmov   q7, q4                   \n\t"*/
/*		"vmov   q8, q4                   \n\t"*/
/*		"vmov   q9, q4                   \n\t"*/
/*		"vmov   q10, q4                  \n\t"*/
/*		"vmov   q11, q4                  \n\t"*/
/*		"vmov   q12, q4                  \n\t"*/
/*		"vmov   q13, q4                  \n\t"*/
/*		"vmov   q14, q4                  \n\t"*/
/*		"vmov   q15, q4                  \n\t"*/
		"                                \n\t"
		"                                \n\t"
		"vmul.f32  q4, q1, d0[0]        \n\t"
		"vldr   d6, [%4, #0]             \n\t"
/*		"vmla.f32  q5, q1, d0[1]        \n\t"*/
		"vldr   d7, [%4, #8]             \n\t"
/*		"vmla.f32  q6, q1, d1[0]        \n\t"*/
		"pld    [%2, #128]                \n\t" // A0
/*		"vmla.f32  q7, q1, d1[1]        \n\t"*/
		"vldr   d2, [%2, #0]             \n\t"
		"                                \n\t"
		"vmul.f32  q8, q2, d0[0]        \n\t"
		"vldr   d3, [%2, #8]             \n\t"
/*		"vmla.f32  q9, q2, d0[1]        \n\t"*/
		"pld    [%3, #128]                \n\t" // A1
/*		"vmla.f32  q10, q2, d1[0]        \n\t"*/
		"pld    [%4, #144]                \n\t" // A2
/*		"vmla.f32  q11, q2, d1[1]        \n\t"*/
		"vldr   d4, [%5, #0]             \n\t"
		"                                \n\t"
		"vmul.f32  q12, q3, d0[0]        \n\t"
		"vldr   d5, [%5, #8]             \n\t"
/*		"vmla.f32  q13, q3, d0[1]        \n\t"*/
		"vldr   d0, [%3, #0]             \n\t"
/*		"vmla.f32  q14, q3, d1[0]        \n\t"*/
		"pld    [%5, #128]                \n\t" // B
/*		"vmla.f32  q15, q3, d1[1]        \n\t"*/
		"vldr   d1, [%3, #8]             \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q4, q1, d4[0]        \n\t"
		"vldr   d6, [%4, #16]             \n\t"
		"vmul.f32  q5, q1, d4[1]        \n\t"
		"vldr   d7, [%4, #24]             \n\t"
/*		"vmla.f32  q6, q1, d5[0]        \n\t"*/
/*		"vmla.f32  q7, q1, d5[1]        \n\t"*/
		"vldr   d2, [%2, #16]             \n\t"
		"                                \n\t"
		"vmla.f32  q8, q0, d4[0]        \n\t"
		"vldr   d3, [%2, #24]             \n\t"
		"vmul.f32  q9, q0, d4[1]        \n\t"
/*		"vmla.f32  q10, q0, d5[0]        \n\t"*/
/*		"vmla.f32  q11, q0, d5[1]        \n\t"*/
		"vldr   d0, [%5, #16]             \n\t"
		"                                \n\t"
		"vmla.f32  q12, q3, d4[0]        \n\t"
		"vldr   d1, [%5, #24]             \n\t"
		"vmul.f32  q13, q3, d4[1]        \n\t"
		"vldr   d4, [%3, #16]             \n\t"
/*		"vmla.f32  q14, q3, d5[0]        \n\t"*/
/*		"vmla.f32  q15, q3, d5[1]        \n\t"*/
		"vldr   d5, [%3, #24]             \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q4, q1, d0[0]        \n\t"
		"vldr   d6, [%4, #32]             \n\t"
		"vmla.f32  q5, q1, d0[1]        \n\t"
		"vldr   d7, [%4, #40]             \n\t"
		"vmul.f32  q6, q1, d1[0]        \n\t"
/*		"vmla.f32  q7, q1, d1[1]        \n\t"*/
		"vldr   d2, [%2, #32]             \n\t"
		"                                \n\t"
		"vmla.f32  q8, q2, d0[0]        \n\t"
		"vldr   d3, [%2, #40]             \n\t"
		"vmla.f32  q9, q2, d0[1]        \n\t"
		"vmul.f32  q10, q2, d1[0]        \n\t"
/*		"vmla.f32  q11, q2, d1[1]        \n\t"*/
		"vldr   d4, [%5, #32]             \n\t"
		"                                \n\t"
		"vmla.f32  q12, q3, d0[0]        \n\t"
		"vldr   d5, [%5, #40]             \n\t"
		"vmla.f32  q13, q3, d0[1]        \n\t"
		"vldr   d0, [%3, #32]             \n\t"
		"vmul.f32  q14, q3, d1[0]        \n\t"
/*		"vmla.f32  q15, q3, d1[1]        \n\t"*/
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
		"vmul.f32  q7, q1, d5[1]        \n\t"
		"vldr   d2, [%2, #48]             \n\t"
		"                                \n\t"
		"vmla.f32  q8, q0, d4[0]        \n\t"
		"vldr   d3, [%2, #56]             \n\t"
		"vmla.f32  q9, q0, d4[1]        \n\t"
		"add    %2, %2, #64              \n\t" // increase B
		"vmla.f32  q10, q0, d5[0]        \n\t"
		"vmul.f32  q11, q0, d5[1]        \n\t"
		"vldr   d0, [%5, #48]             \n\t"
		"                                \n\t"
		"vmla.f32  q12, q3, d4[0]        \n\t"
		"vldr   d1, [%5, #56]             \n\t"
		"vmla.f32  q13, q3, d4[1]        \n\t"
		"vldr   d4, [%3, #48]             \n\t"
		"vmla.f32  q14, q3, d5[0]        \n\t"
		"add    %5, %5, #64              \n\t" // increase B
		"vmul.f32  q15, q3, d5[1]        \n\t"
		"vldr   d5, [%3, #56]             \n\t"
		"add    %3, %3, #64              \n\t" // increase B
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"cmp    r0, #0                   \n\t"
		"                                \n\t"
		"ble    .DCONSIDERLEFT2_12x4           \n\t"
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
		"                                \n\t"
		"cmp    r0, #0                   \n\t"
		"                                \n\t"
		"bgt    .DLOOPKITER_12x4              \n\t"
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
		"                                \n\t"
		"vst1.64   {d8, d9, d10, d11},     [%6:128]!  \n\t" // store D0
		"vst1.64   {d12, d13, d14, d15},   [%6:128]  \n\t" // store D0
		"vst1.64   {d16, d17, d18, d19},   [%7:128]!  \n\t" // store D0
		"vst1.64   {d20, d21, d22, d23},   [%7:128]  \n\t" // store D0
		"vst1.64   {d24, d25, d26, d27},   [%8:128]!  \n\t" // store D0
		"vst1.64   {d28, d29, d30, d31},   [%8:128]  \n\t" // store D0
/*		"                                \n\t"*/
/*		"                                \n\t"*/
		"                                \n\t"
/*		".align 3                        \n\t"*/
/*		".DZERO12:                         \n\t" // zero quad word*/
/*		".word  0                        \n\t"*/
/*		".word  0                        \n\t"*/
/*		".word  0                        \n\t"*/
/*		".word  0                        \n\t"*/
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
		  "r" (C0),			// %6
		  "r" (C1),			// %7
		  "r" (C2)			// %8
		: // register clobber list
		  "r0",
		  "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",
		  "d8", "d9", "d10", "d11", "d12", "d13", "d14", "d15",
		  "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23",
		  "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31",
		  "memory"
	);
}



// normal-transposed, 8x4 with data packed in 4
void kernel_strmm_nt_8x4_lib4(int kmax, float *A0, float *A1, float *B, float *C0, float *C1)
	{
	
/*	if(kmax<=0)*/
/*		return;*/
		
	__builtin_prefetch( A0 );
	__builtin_prefetch( A1 );
	__builtin_prefetch( B  );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A0+8 );
	__builtin_prefetch( A1+8 );
	__builtin_prefetch( B +8 );
#endif

	int k_iter = kmax/4 - 1;
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
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)
		"                                \n\t"
		"vmul.f32  q0, q4, d12[0]        \n\t"
		"vldr   d26, [%3, #16]             \n\t"
/*		"vmla.f32  q1, q4, d12[1]        \n\t"*/
		"vldr   d27, [%3, #24]             \n\t"
/*		"vmla.f32  q14, q4, d13[0]        \n\t"*/
		"pld    [%2, #128]                \n\t"
/*		"vmla.f32  q15, q4, d13[1]        \n\t"*/
		"vldr   d8, [%2, #32]             \n\t"
		"                                \n\t"
		"vmul.f32  q8, q12, d12[0]        \n\t"
		"vldr   d9, [%2, #40]             \n\t"
/*		"vmla.f32  q9, q12, d12[1]        \n\t"*/
		"pld    [%3, #128]                \n\t"
/*		"vmla.f32  q10, q12, d13[0]        \n\t"*/
		"pld    [%4, #128]                \n\t"
/*		"vmla.f32  q11, q12, d13[1]        \n\t"*/
		"vldr   d24, [%3, #32]             \n\t"
		"                                \n\t"
		"vmla.f32  q0, q5, d14[0]        \n\t"
		"vldr   d25, [%3, #40]             \n\t"
		"vmul.f32  q1, q5, d14[1]        \n\t"
		"vldr   d12, [%4, #32]             \n\t"
/*		"vmla.f32  q14, q5, d15[0]        \n\t"*/
		"vldr   d13, [%4, #40]             \n\t"
/*		"vmla.f32  q15, q5, d15[1]        \n\t"*/
		"vldr   d10, [%2, #48]             \n\t"
		"                                \n\t"
		"vmla.f32  q8, q13, d14[0]        \n\t"
		"vldr   d11, [%2, #56]             \n\t"
		"vmul.f32  q9, q13, d14[1]        \n\t"
/*		"vmla.f32  q10, q13, d15[0]        \n\t"*/
		"vldr   d14, [%4, #48]             \n\t"
/*		"vmla.f32  q11, q13, d15[1]        \n\t"*/
		"vldr   d26, [%3, #48]             \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q0, q4, d4[0]        \n\t"
		"vldr   d27, [%3, #56]             \n\t"
		"vmla.f32  q1, q4, d4[1]        \n\t"
		"vldr   d15, [%4, #56]             \n\t"
		"vmul.f32  q14, q4, d5[0]        \n\t"
/*		"vmla.f32  q15, q4, d5[1]        \n\t"*/
		"vldr   d8, [%2, #64]             \n\t"
		"                                \n\t"
		"vmla.f32  q8, q12, d4[0]        \n\t"
		"vldr   d9, [%2, #72]             \n\t"
		"vmla.f32  q9, q12, d4[1]        \n\t"
		"vmul.f32  q10, q12, d5[0]        \n\t"
		"vldr   d4, [%4, #64]             \n\t"
/*		"vmla.f32  q11, q12, d5[1]        \n\t"*/
		"vldr   d24, [%3, #64]             \n\t"
		"                                \n\t"
		"vmla.f32  q0, q5, d6[0]        \n\t"
		"vldr   d25, [%3, #72]             \n\t"
		"vmla.f32  q1, q5, d6[1]        \n\t"
		"vldr   d5, [%4, #72]             \n\t"
		"vmla.f32  q14, q5, d7[0]        \n\t"
		"add    %3, %3, #64              \n\t" // increase A
		"vmul.f32  q15, q5, d7[1]        \n\t"
		"vldr   d10, [%2, #80]             \n\t"
		"                                \n\t"
		"vmla.f32  q8, q13, d6[0]        \n\t"
		"vldr   d11, [%2, #88]             \n\t"
		"vmla.f32  q9, q13, d6[1]        \n\t"
		"add    %2, %2, #64              \n\t" // increase A
		"vmla.f32  q10, q13, d7[0]        \n\t"
		"vldr   d6, [%4, #80]             \n\t"
		"vmul.f32  q11, q13, d7[1]        \n\t"
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
		"vmul.f32  q0, q4, d12[0]        \n\t"
/*		"vmla.f32  q1, q4, d12[1]        \n\t"*/
/*		"vmla.f32  q14, q4, d13[0]        \n\t"*/
/*		"vmla.f32  q15, q4, d13[1]        \n\t"*/
		"                                \n\t"
		"vmul.f32  q8, q12, d12[0]        \n\t"
/*		"vmla.f32  q9, q12, d12[1]        \n\t"*/
/*		"vmla.f32  q10, q12, d13[0]        \n\t"*/
/*		"vmla.f32  q11, q12, d13[1]        \n\t"*/
		"                                \n\t"
		"vmla.f32  q0, q5, d14[0]        \n\t"
		"vmul.f32  q1, q5, d14[1]        \n\t"
/*		"vmla.f32  q14, q5, d15[0]        \n\t"*/
/*		"vmla.f32  q15, q5, d15[1]        \n\t"*/
		"                                \n\t"
		"vmla.f32  q8, q13, d14[0]        \n\t"
		"vmul.f32  q9, q13, d14[1]        \n\t"
/*		"vmla.f32  q10, q13, d15[0]        \n\t"*/
/*		"vmla.f32  q11, q13, d15[1]        \n\t"*/
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
		"vmul.f32  q14, q4, d13[0]        \n\t"
/*		"vmla.f32  q15, q4, d13[1]        \n\t"*/
		"                                \n\t"
		"vmla.f32  q8, q12, d12[0]        \n\t"
		"vmla.f32  q9, q12, d12[1]        \n\t"
		"vmul.f32  q10, q12, d13[0]        \n\t"
/*		"vmla.f32  q11, q12, d13[1]        \n\t"*/
		"                                \n\t"
		"vmla.f32  q0, q5, d14[0]        \n\t"
		"vmla.f32  q1, q5, d14[1]        \n\t"
		"vmla.f32  q14, q5, d15[0]        \n\t"
		"vmul.f32  q15, q5, d15[1]        \n\t"
		"                                \n\t"
		"vmla.f32  q8, q13, d14[0]        \n\t"
		"vmla.f32  q9, q13, d14[1]        \n\t"
		"vmla.f32  q10, q13, d15[0]        \n\t"
		"vmul.f32  q11, q13, d15[1]        \n\t"
		"                                \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%4:128]! \n\t" // load B
		"vld1.64   {d8, d9, d10, d11},   [%2:128]! \n\t" // load A0
		"vld1.64   {d24, d25, d26, d27}, [%3:128]! \n\t" // load A0
		"                                \n\t"
#endif
		"                                \n\t"
		"                                \n\t"
		"cmp    r0, #0                   \n\t"
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
		"cmp    r0, #0                   \n\t" // next iter?
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
/*		"pld    [%2, #128]                \n\t"*/
		"vmla.f32  q15, q4, d13[1]        \n\t"
		"vldr   d8, [%2, #32]             \n\t"
		"                                \n\t"
		"vmla.f32  q8, q12, d12[0]        \n\t"
		"vldr   d9, [%2, #40]             \n\t"
		"vmla.f32  q9, q12, d12[1]        \n\t"
/*		"pld    [%3, #128]                \n\t"*/
		"vmla.f32  q10, q12, d13[0]        \n\t"
/*		"pld    [%4, #128]                \n\t"*/
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
/*		"vldr   d10, [%2, #48]             \n\t"*/
		"                                \n\t"
		"vmla.f32  q8, q13, d14[0]        \n\t"
/*		"vldr   d11, [%2, #56]             \n\t"*/
		"vmla.f32  q9, q13, d14[1]        \n\t"
/*		"sub    r0, r0, #1               \n\t" // iter++*/
		"vmla.f32  q10, q13, d15[0]        \n\t"
/*		"vldr   d14, [%4, #48]             \n\t"*/
		"vmla.f32  q11, q13, d15[1]        \n\t"
/*		"vldr   d26, [%3, #48]             \n\t"*/
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
		"vst1.64   {d0, d1, d2, d3},     [%5:128]!  \n\t" // store C
		"vst1.64   {d4, d5, d6, d7},     [%5:128]   \n\t" // store C
		"vst1.64   {d16, d17, d18, d19}, [%6:128]!  \n\t" // store C
		"vst1.64   {d20, d21, d22, d23}, [%6:128]   \n\t" // store C
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
/*		".align 3                        \n\t"*/
/*		".DZERO4:                         \n\t" // zero quad word*/
/*		".word  0                        \n\t"*/
/*		".word  0                        \n\t"*/
/*		".word  0                        \n\t"*/
/*		".word  0                        \n\t"*/
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
		  "r" (C1)			// %6
		: // register clobber list
		  "r0",
		  "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",
		  "d8", "d9", "d10", "d11", "d12", "d13", "d14", "d15",
		  "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23",
		  "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31",
		  "memory"
	);
}



void kernel_strmm_nt_4x4_lib4(int kmax, float *A, float *B, float *C)
	{
	
/*	if(kmax<=0)*/
/*		return;*/
		
	__builtin_prefetch( A );
	__builtin_prefetch( B );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A+8 );
	__builtin_prefetch( B+8 );
#endif

	int k_iter = kmax/4 - 1;
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
		"                                \n\t"
/*		"vldr   d0, .DZERO_4x4               \n\t" // load zero double*/
/*		"vldr   d1, .DZERO_4x4+8             \n\t" // load zero double*/
/*		"vmov   q1, q0                   \n\t"*/
/*		"vmov   q2, q0                   \n\t"*/
/*		"vmov   q3, q0                   \n\t"*/
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)
		"                                \n\t"
		"pld    [%2, #128]                \n\t"
		"pld    [%3, #128]                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmul.f32  q0, q4, d12[0]        \n\t"
		"vldr   d10, [%2, #16]             \n\t"
/*		"vmla.f32  q1, q4, d12[1]        \n\t"*/
		"vldr   d12, [%3, #32]             \n\t"
/*		"vmla.f32  q2, q4, d13[0]        \n\t"*/
		"vldr   d11, [%2, #24]             \n\t"
/*		"vmla.f32  q3, q4, d13[1]        \n\t"*/
		"vldr   d13, [%3, #40]             \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q0, q5, d14[0]        \n\t"
		"vldr   d8, [%2, #32]             \n\t"
		"vmul.f32  q1, q5, d14[1]        \n\t"
		"vldr   d14, [%3, #48]             \n\t"
/*		"vmla.f32  q2, q5, d15[0]        \n\t"*/
		"vldr   d9, [%2, #40]             \n\t"
/*		"vmla.f32  q3, q5, d15[1]        \n\t"*/
		"vldr   d15, [%3, #56]             \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q0, q4, d12[0]        \n\t"
		"vldr   d10, [%2, #48]             \n\t"
		"vmla.f32  q1, q4, d12[1]        \n\t"
		"vldr   d12, [%3, #64]             \n\t"
		"vmul.f32  q2, q4, d13[0]        \n\t"
		"vldr   d11, [%2, #56]             \n\t"
/*		"vmla.f32  q3, q4, d13[1]        \n\t"*/
		"vldr   d13, [%3, #72]             \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q0, q5, d14[0]        \n\t"
		"vldr   d8, [%2, #64]             \n\t"
		"vmla.f32  q1, q5, d14[1]        \n\t"
		"vldr   d14, [%3, #80]             \n\t"
		"vmla.f32  q2, q5, d15[0]        \n\t"
		"vldr   d9, [%2, #72]             \n\t"
		"vmul.f32  q3, q5, d15[1]        \n\t"
		"vldr   d15, [%3, #88]             \n\t"
		"                                \n\t"
		"add    %2, %2, #64              \n\t" // increase A
		"add    %3, %3, #64              \n\t" // increase A
		"                                \n\t"
		"                                \n\t"
#endif
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)
		"                                \n\t"
		"pld    [%2, #96]                \n\t"
		"pld    [%3, #96]                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmul.f32  q0, q4, d12[0]        \n\t"
/*		"vmla.f32  q1, q4, d12[1]        \n\t"*/
/*		"vmla.f32  q2, q4, d13[0]        \n\t"*/
/*		"vmla.f32  q3, q4, d13[1]        \n\t"*/
		"                                \n\t"
		"vmla.f32  q0, q5, d14[0]        \n\t"
		"vmul.f32  q1, q5, d14[1]        \n\t"
/*		"vmla.f32  q2, q5, d15[0]        \n\t"*/
/*		"vmla.f32  q3, q5, d15[1]        \n\t"*/
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
		"                                \n\t"
		"vmla.f32  q0, q4, d12[0]        \n\t"
		"vmla.f32  q1, q4, d12[1]        \n\t"
		"vmul.f32  q2, q4, d13[0]        \n\t"
/*		"vmla.f32  q3, q4, d13[1]        \n\t"*/
		"                                \n\t"
		"vmla.f32  q0, q5, d14[0]        \n\t"
		"vmla.f32  q1, q5, d14[1]        \n\t"
		"vmla.f32  q2, q5, d15[0]        \n\t"
		"vmul.f32  q3, q5, d15[1]        \n\t"
		"                                \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%3:128]! \n\t" // load B
		"vld1.64   {d8, d9, d10, d11},   [%2:128]! \n\t" // load A0
		"                                \n\t"
#endif
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"cmp    r0, #0                   \n\t"
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
#endif
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)
		"                                \n\t"
		"pld    [%2, #96]                \n\t"
		"pld    [%3, #96]                \n\t"
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
		"sub    r0, r0, #1               \n\t" // iter++
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
		"                                \n\t"
		"vst1.64   {d0, d1, d2, d3}, [%4:128]!  \n\t" // store D
		"vst1.64   {d4, d5, d6, d7}, [%4:128]   \n\t" // store D
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
		  "r" (C)			// %4
		: // register clobber list
		  "r0",
		  "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7",
		  "s8", "s9", "s10", "s11", "s12", "s13", "s14", "s15",
		  "s16", "s17", "s18", "s19", "s20", "s21", "s22", "s23",
		  "s24", "s25", "s26", "s27", "s28", "s29", "s30", "s31",
		  "memory"
	);
}



void corner_strmm_nt_4x3_lib4(float *A, float *B, float *C)
	{

	const int bs = 4;
	const int lda = bs;
	const int ldc = bs;

	float
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



void corner_strmm_nt_4x2_lib4(float *A, float *B, float *C)
	{

	const int bs = 4;
	const int lda = bs;
	const int ldc = bs;

	float
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



void corner_strmm_nt_4x1_lib4(float *A, float *B, float *C)
	{

	const int bs = 4;
	const int lda = bs;
	const int ldc = bs;

	float
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




