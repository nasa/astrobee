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

#include "../../include/block_size.h"



void kernel_sgemm_strsm_nt_12x4_lib4(int kadd, int ksub, float *A0, float *A1, float *A2, float *B, float *C0, float *C1, float *C2, float *D0, float *D1, float *D2, float *fact)
	{
	
/*	if(kmax<=0)*/
/*		return;*/
		
	__builtin_prefetch( A0 );
	__builtin_prefetch( A1 );
	__builtin_prefetch( A2 );
	__builtin_prefetch( B  );

	int ki_add = kadd/4;
	int kl_add = kadd%4;
	int ki_sub = ksub/4;

	const int bs = S_MR;//4;
	const int d_ncl = S_NCL;//2;

	int dA = bs*((d_ncl-kadd%d_ncl)%d_ncl)*sizeof(float);
	
/*	printf("\n%d %d %d %d\n", kadd, ki_add, kl_add, ki_sub);*/

	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"pld    [%3, #64]                \n\t" // prefetch A0 to L1
		"pld    [%4, #64]                \n\t" // prefetch A1 to L1
		"pld    [%5, #64]                \n\t" // prefetch A2 to L1
		"pld    [%6, #64]                \n\t" // prefetch B to L1
		"                                \n\t"
		"                                \n\t"
		"mov    r0, %0                   \n\t" // k_iter
		"                                \n\t"
		"                                \n\t"
		"vld1.64   {d0, d1}, [%6:128]! \n\t" // load B to registers
		"vld1.64   {d2, d3}, [%3:128]! \n\t" // load A0 to registers
		"vld1.64   {d4, d5}, [%4:128]! \n\t" // load A1
/*		"vld1.64   {d6, d7}, [%5:128]! \n\t" // load A2*/
		"                                \n\t"
		"cmp    r0, #0                   \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"ldr    r4, %6                   \n\t" // load address of C*/
/*		"ldr    r5, %6                   \n\t" // load address of C*/
/*		"ldr    r6, %7                   \n\t" // alg*/
		"                                \n\t"
		"                                \n\t"
		"vldr   d8, .DZERO_12x4               \n\t" // load zero double
		"vldr   d9, .DZERO_12x4+8             \n\t" // load zero double
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
		"ble    .DCONSIDERADD2_12x4      \n\t"
		"                                \n\t"
		"                                \n\t"
		"b      .DENDZERO_12x4           \n\t"
		".align 3                        \n\t"
		".DZERO_12x4:                    \n\t" // zero quad word
		".word  0                        \n\t"
		".word  0                        \n\t"
		".word  0                        \n\t"
		".word  0                        \n\t"
		".DENDZERO_12x4:                 \n\t"
		"                                \n\t"
		"                                \n\t"
		".DLOOPADD_12x4:                \n\t" // main loop
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q4, q1, d0[0]        \n\t"
		"vldr   d6, [%5, #0]             \n\t"
		"vmla.f32  q5, q1, d0[1]        \n\t"
		"vldr   d7, [%5, #8]             \n\t"
		"vmla.f32  q6, q1, d1[0]        \n\t"
		"pld    [%3, #128]                \n\t" // A0
		"vmla.f32  q7, q1, d1[1]        \n\t"
		"vldr   d2, [%3, #0]             \n\t"
		"                                \n\t"
		"vmla.f32  q8, q2, d0[0]        \n\t"
		"vldr   d3, [%3, #8]             \n\t"
		"vmla.f32  q9, q2, d0[1]        \n\t"
		"pld    [%4, #128]                \n\t" // A1
		"vmla.f32  q10, q2, d1[0]        \n\t"
		"pld    [%5, #144]                \n\t" // A2
		"vmla.f32  q11, q2, d1[1]        \n\t"
		"vldr   d4, [%6, #0]             \n\t"
		"                                \n\t"
		"vmla.f32  q12, q3, d0[0]        \n\t"
		"vldr   d5, [%6, #8]             \n\t"
		"vmla.f32  q13, q3, d0[1]        \n\t"
		"vldr   d0, [%4, #0]             \n\t"
		"vmla.f32  q14, q3, d1[0]        \n\t"
		"pld    [%6, #128]                \n\t" // B
		"vmla.f32  q15, q3, d1[1]        \n\t"
		"vldr   d1, [%4, #8]             \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q4, q1, d4[0]        \n\t"
		"vldr   d6, [%5, #16]             \n\t"
		"vmla.f32  q5, q1, d4[1]        \n\t"
		"vldr   d7, [%5, #24]             \n\t"
		"vmla.f32  q6, q1, d5[0]        \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"vmla.f32  q7, q1, d5[1]        \n\t"
		"vldr   d2, [%3, #16]             \n\t"
		"                                \n\t"
		"vmla.f32  q8, q0, d4[0]        \n\t"
		"vldr   d3, [%3, #24]             \n\t"
		"vmla.f32  q9, q0, d4[1]        \n\t"
		"vmla.f32  q10, q0, d5[0]        \n\t"
		"vmla.f32  q11, q0, d5[1]        \n\t"
		"vldr   d0, [%6, #16]             \n\t"
		"                                \n\t"
		"vmla.f32  q12, q3, d4[0]        \n\t"
		"vldr   d1, [%6, #24]             \n\t"
		"vmla.f32  q13, q3, d4[1]        \n\t"
		"vldr   d4, [%4, #16]             \n\t"
		"vmla.f32  q14, q3, d5[0]        \n\t"
		"cmp    r0, #0                   \n\t" // next iter?
		"vmla.f32  q15, q3, d5[1]        \n\t"
		"vldr   d5, [%4, #24]             \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q4, q1, d0[0]        \n\t"
		"vldr   d6, [%5, #32]             \n\t"
		"vmla.f32  q5, q1, d0[1]        \n\t"
		"vldr   d7, [%5, #40]             \n\t"
		"vmla.f32  q6, q1, d1[0]        \n\t"
		"vmla.f32  q7, q1, d1[1]        \n\t"
		"vldr   d2, [%3, #32]             \n\t"
		"                                \n\t"
		"vmla.f32  q8, q2, d0[0]        \n\t"
		"vldr   d3, [%3, #40]             \n\t"
		"vmla.f32  q9, q2, d0[1]        \n\t"
		"vmla.f32  q10, q2, d1[0]        \n\t"
		"vmla.f32  q11, q2, d1[1]        \n\t"
		"vldr   d4, [%6, #32]             \n\t"
		"                                \n\t"
		"vmla.f32  q12, q3, d0[0]        \n\t"
		"vldr   d5, [%6, #40]             \n\t"
		"vmla.f32  q13, q3, d0[1]        \n\t"
		"vldr   d0, [%4, #32]             \n\t"
		"vmla.f32  q14, q3, d1[0]        \n\t"
		"vmla.f32  q15, q3, d1[1]        \n\t"
		"vldr   d1, [%4, #40]             \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q4, q1, d4[0]        \n\t"
		"vldr   d6, [%5, #48]             \n\t"
		"vmla.f32  q5, q1, d4[1]        \n\t"
		"vldr   d7, [%5, #56]             \n\t"
		"vmla.f32  q6, q1, d5[0]        \n\t"
		"add    %5, %5, #64              \n\t" // increase B
		"vmla.f32  q7, q1, d5[1]        \n\t"
		"vldr   d2, [%3, #48]             \n\t"
		"                                \n\t"
		"vmla.f32  q8, q0, d4[0]        \n\t"
		"vldr   d3, [%3, #56]             \n\t"
		"vmla.f32  q9, q0, d4[1]        \n\t"
		"add    %3, %3, #64              \n\t" // increase B
		"vmla.f32  q10, q0, d5[0]        \n\t"
		"vmla.f32  q11, q0, d5[1]        \n\t"
		"vldr   d0, [%6, #48]             \n\t"
		"                                \n\t"
		"vmla.f32  q12, q3, d4[0]        \n\t"
		"vldr   d1, [%6, #56]             \n\t"
		"vmla.f32  q13, q3, d4[1]        \n\t"
		"vldr   d4, [%4, #48]             \n\t"
		"vmla.f32  q14, q3, d5[0]        \n\t"
		"add    %6, %6, #64              \n\t" // increase B
		"vmla.f32  q15, q3, d5[1]        \n\t"
		"vldr   d5, [%4, #56]             \n\t"
		"add    %4, %4, #64              \n\t" // increase B
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"bgt    .DLOOPADD_12x4              \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERADD2_12x4:                 \n\t" // consider left k+=2
		"                                \n\t"
		"mov    r0, %1                   \n\t" // k_left
		"cmp    r0, #1                   \n\t"
		"ble    .DCONSIDERADD1_12x4          \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q4, q1, d0[0]        \n\t"
		"vldr   d6, [%5, #0]             \n\t"
		"vmla.f32  q5, q1, d0[1]        \n\t"
		"vldr   d7, [%5, #8]             \n\t"
		"vmla.f32  q6, q1, d1[0]        \n\t"
		"vmla.f32  q7, q1, d1[1]        \n\t"
		"vldr   d2, [%3, #0]             \n\t"
		"                                \n\t"
		"vmla.f32  q8, q2, d0[0]        \n\t"
		"vldr   d3, [%3, #8]             \n\t"
		"vmla.f32  q9, q2, d0[1]        \n\t"
		"vmla.f32  q10, q2, d1[0]        \n\t"
		"vmla.f32  q11, q2, d1[1]        \n\t"
		"vldr   d4, [%6, #0]             \n\t"
		"                                \n\t"
		"vmla.f32  q12, q3, d0[0]        \n\t"
		"vldr   d5, [%6, #8]             \n\t"
		"vmla.f32  q13, q3, d0[1]        \n\t"
		"vldr   d0, [%4, #0]             \n\t"
		"vmla.f32  q14, q3, d1[0]        \n\t"
		"vmla.f32  q15, q3, d1[1]        \n\t"
		"vldr   d1, [%4, #8]             \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q4, q1, d4[0]        \n\t"
		"vldr   d6, [%5, #16]             \n\t"
		"vmla.f32  q5, q1, d4[1]        \n\t"
		"vldr   d7, [%5, #24]             \n\t"
		"vmla.f32  q6, q1, d5[0]        \n\t"
		"add    %5, %5, #32              \n\t" // increase B
		"vmla.f32  q7, q1, d5[1]        \n\t"
		"vldr   d2, [%3, #16]             \n\t"
		"                                \n\t"
		"vmla.f32  q8, q0, d4[0]        \n\t"
		"vldr   d3, [%3, #24]             \n\t"
		"vmla.f32  q9, q0, d4[1]        \n\t"
		"add    %3, %3, #32              \n\t" // increase B
		"vmla.f32  q10, q0, d5[0]        \n\t"
		"vmla.f32  q11, q0, d5[1]        \n\t"
		"vldr   d0, [%6, #16]             \n\t"
		"                                \n\t"
		"vmla.f32  q12, q3, d4[0]        \n\t"
		"vldr   d1, [%6, #24]             \n\t"
		"vmla.f32  q13, q3, d4[1]        \n\t"
		"vldr   d4, [%4, #16]             \n\t"
		"vmla.f32  q14, q3, d5[0]        \n\t"
		"add    %6, %6, #32              \n\t" // increase B
		"vmla.f32  q15, q3, d5[1]        \n\t"
		"vldr   d5, [%4, #24]             \n\t"
		"add    %4, %4, #32              \n\t" // increase B
		"                                \n\t"
		"sub    r0, r0, #2               \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERADD1_12x4:                 \n\t" // consider left k++
		"                                \n\t"
		"cmp    r0, #0                   \n\t"
		"ble    .DCONSIDERSUB_12x4              \n\t"
		"                                \n\t"
		"vmla.f32  q4, q1, d0[0]        \n\t"
		"vldr   d6, [%5, #0]             \n\t"
		"vmla.f32  q5, q1, d0[1]        \n\t"
		"vldr   d7, [%5, #8]             \n\t"
		"vmla.f32  q6, q1, d1[0]        \n\t"
		"add    %3, %3, #16              \n\t" // increase B
		"vmla.f32  q7, q1, d1[1]        \n\t"
		"                                \n\t"
		"vmla.f32  q8, q2, d0[0]        \n\t"
		"add    %4, %4, #16              \n\t" // increase B
		"vmla.f32  q9, q2, d0[1]        \n\t"
		"add    %5, %5, #16              \n\t" // increase B
		"vmla.f32  q10, q2, d1[0]        \n\t"
		"add    %6, %6, #16              \n\t" // increase B
		"vmla.f32  q11, q2, d1[1]        \n\t"
		"                                \n\t"
		"vmla.f32  q12, q3, d0[0]        \n\t"
		"vmla.f32  q13, q3, d0[1]        \n\t"
		"vmla.f32  q14, q3, d1[0]        \n\t"
		"vmla.f32  q15, q3, d1[1]        \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERSUB_12x4:                    \n\t"
		"                                \n\t"
		"add    r0, %0, %1               \n\t"
		"                                \n\t"
		"cmp    %2, #0                   \n\t"
		"ble    .DPOSTACC_12x4                \n\t"
		"                                \n\t"
		"cmp    %14, #0                   \n\t"
		"ble    .DPRELOOPSUB_12x4             \n\t"
		"                                \n\t"
		"cmp    r0, #0                   \n\t"
		"ble    .DPRELOOPSUB_12x4             \n\t"
		"                                \n\t"
		"add    %3, %3, %14               \n\t"
		"add    %4, %4, %14               \n\t"
		"add    %5, %5, %14               \n\t"
		"add    %6, %6, %14               \n\t"
		"                                \n\t"
		"                                \n\t"
		".DPRELOOPSUB_12x4:                   \n\t" // 
		"                                \n\t"
		"sub    %3, %3, #16              \n\t" // 
		"sub    %4, %4, #16              \n\t" // 
		"sub    %6, %6, #16              \n\t" // 
		"                                \n\t"
// TODO load in clean-up loops and not load again if not increased ???
// TODO load already increased in clean-up loops ???
		"                                \n\t"
		"pld    [%3, #0]                \n\t"
		"pld    [%4, #0]                \n\t"
		"pld    [%5, #0]                \n\t"
		"pld    [%5, #0]                \n\t"
		"                                \n\t"
		"vld1.64   {d0, d1}, [%6:128]! \n\t" // load B to registers
		"vld1.64   {d2, d3}, [%3:128]! \n\t" // load A0 to registers
		"vld1.64   {d4, d5}, [%4:128]! \n\t" // load A1
/*		"vld1.64   {d6, d7}, [%5:128]! \n\t" // load A2*/
		"                                \n\t"
		"pld    [%3, #48]                \n\t" // prefetch A0 to L1
		"pld    [%4, #48]                \n\t" // prefetch A1 to L1
		"pld    [%5, #64]                \n\t" // prefetch B to L1
		"pld    [%5, #48]                \n\t" // prefetch B to L1
		"                                \n\t"
		"                                \n\t"
		"mov    r0, %2                   \n\t" // k_iter
		"                                \n\t"
		"                                \n\t"
		".DLOOPSUB_12x4:                      \n\t" // main loop 2
		"                                \n\t"
		"vmls.f32  q4, q1, d0[0]        \n\t"
		"vldr   d6, [%5, #0]             \n\t"
		"vmls.f32  q5, q1, d0[1]        \n\t"
		"vldr   d7, [%5, #8]             \n\t"
		"vmls.f32  q6, q1, d1[0]        \n\t"
		"pld    [%3, #128]                \n\t" // A0
		"vmls.f32  q7, q1, d1[1]        \n\t"
		"vldr   d2, [%3, #0]             \n\t"
		"                                \n\t"
		"vmls.f32  q8, q2, d0[0]        \n\t"
		"vldr   d3, [%3, #8]             \n\t"
		"vmls.f32  q9, q2, d0[1]        \n\t"
		"pld    [%4, #128]                \n\t" // A1
		"vmls.f32  q10, q2, d1[0]        \n\t"
		"pld    [%5, #144]                \n\t" // A2
		"vmls.f32  q11, q2, d1[1]        \n\t"
		"vldr   d4, [%6, #0]             \n\t"
		"                                \n\t"
		"vmls.f32  q12, q3, d0[0]        \n\t"
		"vldr   d5, [%6, #8]             \n\t"
		"vmls.f32  q13, q3, d0[1]        \n\t"
		"vldr   d0, [%4, #0]             \n\t"
		"vmls.f32  q14, q3, d1[0]        \n\t"
		"pld    [%6, #128]                \n\t" // B
		"vmls.f32  q15, q3, d1[1]        \n\t"
		"vldr   d1, [%4, #8]             \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmls.f32  q4, q1, d4[0]        \n\t"
		"vldr   d6, [%5, #16]             \n\t"
		"vmls.f32  q5, q1, d4[1]        \n\t"
		"vldr   d7, [%5, #24]             \n\t"
		"vmls.f32  q6, q1, d5[0]        \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"vmls.f32  q7, q1, d5[1]        \n\t"
		"vldr   d2, [%3, #16]             \n\t"
		"                                \n\t"
		"vmls.f32  q8, q0, d4[0]        \n\t"
		"vldr   d3, [%3, #24]             \n\t"
		"vmls.f32  q9, q0, d4[1]        \n\t"
		"vmls.f32  q10, q0, d5[0]        \n\t"
		"vmls.f32  q11, q0, d5[1]        \n\t"
		"vldr   d0, [%6, #16]             \n\t"
		"                                \n\t"
		"vmls.f32  q12, q3, d4[0]        \n\t"
		"vldr   d1, [%6, #24]             \n\t"
		"vmls.f32  q13, q3, d4[1]        \n\t"
		"vldr   d4, [%4, #16]             \n\t"
		"vmls.f32  q14, q3, d5[0]        \n\t"
		"cmp    r0, #0                   \n\t" // next iter?
		"vmls.f32  q15, q3, d5[1]        \n\t"
		"vldr   d5, [%4, #24]             \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmls.f32  q4, q1, d0[0]        \n\t"
		"vldr   d6, [%5, #32]             \n\t"
		"vmls.f32  q5, q1, d0[1]        \n\t"
		"vldr   d7, [%5, #40]             \n\t"
		"vmls.f32  q6, q1, d1[0]        \n\t"
		"vmls.f32  q7, q1, d1[1]        \n\t"
		"vldr   d2, [%3, #32]             \n\t"
		"                                \n\t"
		"vmls.f32  q8, q2, d0[0]        \n\t"
		"vldr   d3, [%3, #40]             \n\t"
		"vmls.f32  q9, q2, d0[1]        \n\t"
		"vmls.f32  q10, q2, d1[0]        \n\t"
		"vmls.f32  q11, q2, d1[1]        \n\t"
		"vldr   d4, [%6, #32]             \n\t"
		"                                \n\t"
		"vmls.f32  q12, q3, d0[0]        \n\t"
		"vldr   d5, [%6, #40]             \n\t"
		"vmls.f32  q13, q3, d0[1]        \n\t"
		"vldr   d0, [%4, #32]             \n\t"
		"vmls.f32  q14, q3, d1[0]        \n\t"
		"vmls.f32  q15, q3, d1[1]        \n\t"
		"vldr   d1, [%4, #40]             \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmls.f32  q4, q1, d4[0]        \n\t"
		"vldr   d6, [%5, #48]             \n\t"
		"vmls.f32  q5, q1, d4[1]        \n\t"
		"vldr   d7, [%5, #56]             \n\t"
		"vmls.f32  q6, q1, d5[0]        \n\t"
		"add    %5, %5, #64              \n\t" // increase B
		"vmls.f32  q7, q1, d5[1]        \n\t"
		"vldr   d2, [%3, #48]             \n\t"
		"                                \n\t"
		"vmls.f32  q8, q0, d4[0]        \n\t"
		"vldr   d3, [%3, #56]             \n\t"
		"vmls.f32  q9, q0, d4[1]        \n\t"
		"add    %3, %3, #64              \n\t" // increase B
		"vmls.f32  q10, q0, d5[0]        \n\t"
		"vmls.f32  q11, q0, d5[1]        \n\t"
		"vldr   d0, [%6, #48]             \n\t"
		"                                \n\t"
		"vmls.f32  q12, q3, d4[0]        \n\t"
		"vldr   d1, [%6, #56]             \n\t"
		"vmls.f32  q13, q3, d4[1]        \n\t"
		"vldr   d4, [%4, #48]             \n\t"
		"vmls.f32  q14, q3, d5[0]        \n\t"
		"add    %6, %6, #64              \n\t" // increase B
		"vmls.f32  q15, q3, d5[1]        \n\t"
		"vldr   d5, [%4, #56]             \n\t"
		"add    %4, %4, #64              \n\t" // increase B
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"bgt    .DLOOPSUB_12x4              \n\t"
		"                                \n\t"
		"                                \n\t"
		".DPOSTACC_12x4:                  \n\t"
		"                                \n\t"
		"                                \n\t"
		"ldr    r1, %7                   \n\t"
		"ldr    r2, %8                   \n\t"
		"ldr    r3, %9                   \n\t"
		"                                \n\t"
		"pld    [r1, #0]                \n\t" // C0
		"pld    [r2, #0]                \n\t" // C1
		"pld    [r3, #0]                \n\t" // C2
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"// alg==-1
		"vld1.64   {d0, d1, d2, d3},   [r1:128]! \n\t" // load C0
		"vld1.64   {d4, d5, d6, d7},   [r1:128]  \n\t" // load C0
		"vadd.f32  q4, q0, q4            \n\t"
		"vadd.f32  q5, q1, q5            \n\t"
		"vld1.64   {d0, d1, d2, d3},   [r2:128]! \n\t" // load C0
		"vadd.f32  q6, q2, q6            \n\t"
		"vadd.f32  q7, q3, q7            \n\t"
		"vld1.64   {d4, d5, d6, d7},   [r2:128]  \n\t" // load C0
		"vadd.f32  q8, q0, q8            \n\t"
		"vadd.f32  q9, q1, q9            \n\t"
		"vld1.64   {d0, d1, d2, d3},   [r3:128]! \n\t" // load C0
		"vadd.f32  q10, q2, q10            \n\t"
		"vadd.f32  q11, q3, q11            \n\t"
		"vld1.64   {d4, d5, d6, d7},   [r3:128]  \n\t" // load C0
		"vadd.f32  q12, q0, q12            \n\t"
		"vadd.f32  q13, q1, q13            \n\t"
		"vadd.f32  q14, q2, q14            \n\t"
		"vadd.f32  q15, q3, q15            \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vld1.64   {d0, d1, d2, d3},   [%13]! \n\t" // fact
		"vld1.64   {d4},   [%13]! \n\t" // fact
		"                                \n\t"
		"                                \n\t"
		"ldr    r1, %10                   \n\t"
		"ldr    r2, %11                   \n\t"
		"ldr    r3, %12                   \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmul.f32  q4, q4, d0[0]         \n\t"
		"vmul.f32  q8, q8, d0[0]         \n\t"
		"vmul.f32  q12, q12, d0[0]         \n\t"
		"                                \n\t"
		"vst1.64   {d8, d9},   [r1:128]! \n\t" // store D0
		"vst1.64   {d16, d17}, [r2:128]! \n\t" // store D1
		"vst1.64   {d24, d25}, [r3:128]! \n\t" // store D1
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmls.f32  q5, q4, d0[1]         \n\t"
		"vmls.f32  q9, q8, d0[1]         \n\t"
		"vmls.f32  q13, q12, d0[1]         \n\t"
		"                                \n\t"
		"vmul.f32  q5, q5, d1[0]         \n\t"
		"vmul.f32  q9, q9, d1[0]         \n\t"
		"vmul.f32  q13, q13, d1[0]         \n\t"
		"                                \n\t"
		"vst1.64   {d10, d11}, [r1:128]! \n\t" // store D0
		"vst1.64   {d18, d19}, [r2:128]! \n\t" // store D1
		"vst1.64   {d26, d27}, [r3:128]! \n\t" // store D1
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmls.f32  q6, q4, d1[1]         \n\t"
		"vmls.f32  q10, q8, d1[1]         \n\t"
		"vmls.f32  q14, q12, d1[1]         \n\t"
		"                                \n\t"
		"vmls.f32  q6, q5, d2[0]         \n\t"
		"vmls.f32  q10, q9, d2[0]         \n\t"
		"vmls.f32  q14, q13, d2[0]         \n\t"
		"                                \n\t"
		"vmul.f32  q6, q6, d2[1]         \n\t"
		"vmul.f32  q10, q10, d2[1]       \n\t"
		"vmul.f32  q14, q14, d2[1]       \n\t"
		"                                \n\t"
		"vst1.64   {d12, d13}, [r1:128]! \n\t" // store D0
		"vst1.64   {d20, d21}, [r2:128]! \n\t" // store D1
		"vst1.64   {d28, d29}, [r3:128]! \n\t" // store D1
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmls.f32  q7, q4, d3[0]         \n\t"
		"vmls.f32  q11, q8, d3[0]         \n\t"
		"vmls.f32  q15, q12, d3[0]         \n\t"
		"                                \n\t"
		"vmls.f32  q7, q5, d3[1]         \n\t"
		"vmls.f32  q11, q9, d3[1]         \n\t"
		"vmls.f32  q15, q13, d3[1]         \n\t"
		"                                \n\t"
		"vmls.f32  q7, q6, d4[0]         \n\t"
		"vmls.f32  q11, q10, d4[0]         \n\t"
		"vmls.f32  q15, q14, d4[0]         \n\t"
		"                                \n\t"
		"vmul.f32  q7, q7, d4[1]         \n\t"
		"vmul.f32  q11, q11, d4[1]         \n\t"
		"vmul.f32  q15, q15, d4[1]         \n\t"
		"                                \n\t"
		"vst1.64   {d14, d15}, [r1:128]! \n\t" // store D0
		"vst1.64   {d22, d23}, [r2:128]! \n\t" // store D1
		"vst1.64   {d30, d31}, [r3:128]! \n\t" // store D1
		"                                \n\t"
		"                                \n\t"
		".DEND_12x4:                      \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		: // output operands (none)
		: // input operands
		  "r" (ki_add),		// %0
		  "r" (kl_add),		// %1
		  "r" (ki_sub),		// %2
		  "r" (A0),			// %3
		  "r" (A1),			// %4
		  "r" (A2),			// %5
		  "r" (B),			// %6
		  "m" (C0),			// %7
		  "m" (C1),			// %8
		  "m" (C2),			// %9
		  "m" (D0),			// %10
		  "m" (D1),			// %11
		  "m" (D2),			// %12
		  "r" (fact),		// %13
		  "r" (dA)			// %14
		: // register clobber list
		  "r0", "r1", "r2", "r3", 
		  "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",
		  "d8", "d9", "d10", "d11", "d12", "d13", "d14", "d15",
		  "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23",
		  "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31",
		  "memory"
	);
}



void kernel_sgemm_strsm_nt_8x4_lib4(int kadd, int ksub, float *A0, float *A1, float *B, float *C0, float *C1, float *D0, float *D1, float *fact)
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

	int ki_add = kadd/4;
	int kl_add = kadd%4;
	int ki_sub = ksub/4;

	const int bs = S_MR;//4;
	const int d_ncl = S_NCL;//2;

	int dA = bs*((d_ncl-kadd%d_ncl)%d_ncl)*sizeof(float);
	
/*	printf("\n%d %d %d\n", kadd, ksub, dA);*/

	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)
		"pld    [%3, #64]                \n\t" // prefetch A0 to L1
		"pld    [%4, #64]                \n\t" // prefetch A1 to L1
		"pld    [%5, #64]                \n\t" // prefetch B to L1
#endif
		"                                \n\t"
		"                                \n\t"
		"mov    r0, %0                   \n\t" // k_iter
		"                                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)
		"vld1.64   {d12, d13, d14, d15}, [%5:128]! \n\t" // load B to registers
		"vld1.64   {d4, d5, d6, d7},     [%5:128] \n\t" // load B to registers
		"vld1.64   {d8, d9, d10, d11},   [%3:128] \n\t" // load A0 to registers
		"vld1.64   {d24, d25},           [%4:128] \n\t" // load A1
#endif
		"                                \n\t"
		"cmp    r0, #0                   \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vldr   d0, .DZERO_8x4               \n\t" // load zero double
		"vldr   d1, .DZERO_8x4+8             \n\t" // load zero double
		"vmov   q1, q0                   \n\t"
		"vmov   q14, q0                   \n\t"
		"vmov   q15, q0                   \n\t"
		"vmov   q8, q0                   \n\t"
		"vmov   q9, q0                   \n\t"
		"vmov   q10, q0                  \n\t"
		"vmov   q11, q0                  \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"ble    .DCONSIDERADD2_8x4           \n\t"
		"                                \n\t"
		"                                \n\t"
		"b      .DENDZERO_8x4                \n\t"
		".align 3                        \n\t"
		".DZERO_8x4:                         \n\t" // zero quad word
		".word  0                        \n\t"
		".word  0                        \n\t"
		".word  0                        \n\t"
		".word  0                        \n\t"
		".DENDZERO_8x4:                      \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DLOOPADD_8x4:                    \n\t" // main loop
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)
		"                                \n\t"
		"vmla.f32  q0, q4, d12[0]        \n\t"
		"vldr   d26, [%4, #16]             \n\t"
		"vmla.f32  q1, q4, d12[1]        \n\t"
		"vldr   d27, [%4, #24]             \n\t"
		"vmla.f32  q14, q4, d13[0]        \n\t"
		"pld    [%3, #128]                \n\t"
		"vmla.f32  q15, q4, d13[1]        \n\t"
		"vldr   d8, [%3, #32]             \n\t"
		"                                \n\t"
		"vmla.f32  q8, q12, d12[0]        \n\t"
		"vldr   d9, [%3, #40]             \n\t"
		"vmla.f32  q9, q12, d12[1]        \n\t"
		"pld    [%4, #128]                \n\t"
		"vmla.f32  q10, q12, d13[0]        \n\t"
		"pld    [%5, #96]                \n\t"
		"vmla.f32  q11, q12, d13[1]        \n\t"
		"vldr   d24, [%4, #32]             \n\t"
		"                                \n\t"
		"vmla.f32  q0, q5, d14[0]        \n\t"
		"vldr   d25, [%4, #40]             \n\t"
		"vmla.f32  q1, q5, d14[1]        \n\t"
		"vldr   d12, [%5, #32]             \n\t"
		"vmla.f32  q14, q5, d15[0]        \n\t"
		"vldr   d13, [%5, #40]             \n\t"
		"vmla.f32  q15, q5, d15[1]        \n\t"
		"vldr   d10, [%3, #48]             \n\t"
		"                                \n\t"
		"vmla.f32  q8, q13, d14[0]        \n\t"
		"vldr   d11, [%3, #56]             \n\t"
		"vmla.f32  q9, q13, d14[1]        \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"vmla.f32  q10, q13, d15[0]        \n\t"
		"vldr   d14, [%5, #48]             \n\t"
		"vmla.f32  q11, q13, d15[1]        \n\t"
		"vldr   d26, [%4, #48]             \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q0, q4, d4[0]        \n\t"
		"vldr   d27, [%4, #56]             \n\t"
		"vmla.f32  q1, q4, d4[1]        \n\t"
		"vldr   d15, [%5, #56]             \n\t"
		"vmla.f32  q14, q4, d5[0]        \n\t"
		"cmp    r0, #0                   \n\t" // next iter?
		"vmla.f32  q15, q4, d5[1]        \n\t"
		"vldr   d8, [%3, #64]             \n\t"
		"                                \n\t"
		"vmla.f32  q8, q12, d4[0]        \n\t"
		"vldr   d9, [%3, #72]             \n\t"
		"vmla.f32  q9, q12, d4[1]        \n\t"
		"vmla.f32  q10, q12, d5[0]        \n\t"
		"vldr   d4, [%5, #64]             \n\t"
		"vmla.f32  q11, q12, d5[1]        \n\t"
		"vldr   d24, [%4, #64]             \n\t"
		"                                \n\t"
		"vmla.f32  q0, q5, d6[0]        \n\t"
		"vldr   d25, [%4, #72]             \n\t"
		"vmla.f32  q1, q5, d6[1]        \n\t"
		"vldr   d5, [%5, #72]             \n\t"
		"vmla.f32  q14, q5, d7[0]        \n\t"
		"add    %4, %4, #64              \n\t" // increase A
		"vmla.f32  q15, q5, d7[1]        \n\t"
		"vldr   d10, [%3, #80]             \n\t"
		"                                \n\t"
		"vmla.f32  q8, q13, d6[0]        \n\t"
		"vldr   d11, [%3, #88]             \n\t"
		"vmla.f32  q9, q13, d6[1]        \n\t"
		"add    %3, %3, #64              \n\t" // increase A
		"vmla.f32  q10, q13, d7[0]        \n\t"
		"vldr   d6, [%5, #80]             \n\t"
		"vmla.f32  q11, q13, d7[1]        \n\t"
		"vldr   d7, [%5, #88]             \n\t"
		"add    %5, %5, #64              \n\t" // increase A
		"                                \n\t"
#endif
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)
		"                                \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%5:128]! \n\t" // load B
		"vld1.64   {d8, d9, d10, d11},   [%3:128]! \n\t" // load A0
		"vld1.64   {d24, d25, d26, d27}, [%4:128]! \n\t" // load A0
		"                                \n\t"
		"pld    [%3, #32]                \n\t"
		"pld    [%4, #32]                \n\t"
		"pld    [%5, #32]                \n\t"
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
		"                                \n\t"
		"                                \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%5:128]! \n\t" // load B
		"vld1.64   {d8, d9, d10, d11},   [%3:128]! \n\t" // load A0
		"vld1.64   {d24, d25, d26, d27}, [%4:128]! \n\t" // load A0
		"                                \n\t"
#if defined(TARGET_CORTEX_A9)
		"pld    [%3, #32]                \n\t"
		"pld    [%4, #32]                \n\t"
		"pld    [%5, #32]                \n\t"
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
#endif
		"                                \n\t"
		"bgt    .DLOOPADD_8x4              \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERADD2_8x4:                 \n\t" // consider left k+=2
		"                                \n\t"
		"mov    r0, %1                   \n\t" // k_left
		"cmp    r0, #1                   \n\t"
		"ble    .DCONSIDERADD1_8x4          \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)
		"                                \n\t"
		"vmla.f32  q0, q4, d12[0]        \n\t"
		"vldr   d26, [%4, #16]             \n\t"
		"vmla.f32  q1, q4, d12[1]        \n\t"
		"vldr   d27, [%4, #24]             \n\t"
		"vmla.f32  q14, q4, d13[0]        \n\t"
		"vmla.f32  q15, q4, d13[1]        \n\t"
		"vldr   d8, [%3, #32]             \n\t"
		"                                \n\t"
		"vmla.f32  q8, q12, d12[0]        \n\t"
		"vldr   d9, [%3, #40]             \n\t"
		"vmla.f32  q9, q12, d12[1]        \n\t"
		"vmla.f32  q10, q12, d13[0]        \n\t"
		"vmla.f32  q11, q12, d13[1]        \n\t"
		"vldr   d24, [%4, #32]             \n\t"
		"                                \n\t"
		"vmla.f32  q0, q5, d14[0]        \n\t"
		"vldr   d25, [%4, #40]             \n\t"
		"vmla.f32  q1, q5, d14[1]        \n\t"
/*		"vldr   d12, [%5, #32]             \n\t"*/
		"vmov   d12, d4                  \n\t"
		"vmla.f32  q14, q5, d15[0]        \n\t"
/*		"vldr   d13, [%5, #40]             \n\t"*/
		"vmov   d13, d5                  \n\t"
		"vmla.f32  q15, q5, d15[1]        \n\t"
		"                                \n\t"
		"vmla.f32  q8, q13, d14[0]        \n\t"
		"add    %3, %3, #32              \n\t" // increase A
		"vmla.f32  q9, q13, d14[1]        \n\t"
		"add    %4, %4, #32              \n\t" // increase A
		"vmla.f32  q10, q13, d15[0]        \n\t"
		"add    %5, %5, #32              \n\t" // increase A
		"vmla.f32  q11, q13, d15[1]        \n\t"
		"                                \n\t"
#endif
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)
		"                                \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%5:128]! \n\t" // load B
		"vld1.64   {d8, d9, d10, d11},   [%3:128]! \n\t" // load A0
		"vld1.64   {d24, d25, d26, d27}, [%4:128]! \n\t" // load A0
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
#endif
		"                                \n\t"
		"sub    r0, r0, #2               \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERADD1_8x4:                 \n\t" // consider left k++
		"                                \n\t"
		"cmp    r0, #0                   \n\t"
		"ble    .DCONSIDERSUB_8x4              \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)
		"                                \n\t"
		"vld1.64   {d12, d13}, [%5:128]! \n\t" // load B
		"vld1.64   {d8, d9},   [%3:128]! \n\t" // load A0
		"vld1.64   {d24, d25}, [%4:128]! \n\t" // load A0
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
#endif
#if defined(TARGET_CORTEX_A15)
		"                                \n\t"
		"vmla.f32  q0, q4, d12[0]        \n\t"
		"vmla.f32  q1, q4, d12[1]        \n\t"
		"vmla.f32  q14, q4, d13[0]        \n\t"
		"vmla.f32  q15, q4, d13[1]        \n\t"
		"                                \n\t"
		"vmla.f32  q8, q12, d12[0]        \n\t"
		"add    %3, %3, #16              \n\t" // increase A
		"vmla.f32  q9, q12, d12[1]        \n\t"
		"add    %4, %4, #16              \n\t" // increase A
		"vmla.f32  q10, q12, d13[0]        \n\t"
		"add    %5, %5, #16              \n\t" // increase A
		"vmla.f32  q11, q12, d13[1]        \n\t"
		"                                \n\t"
#endif
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERSUB_8x4:                    \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"add    r0, %0, %1               \n\t"
		"                                \n\t"
		"cmp    %2, #0                   \n\t"
		"ble    .DPOSTACC_8x4                \n\t"
		"                                \n\t"
		"cmp    %11, #0                   \n\t"
		"ble    .DPRELOOPSUB_8x4             \n\t"
		"                                \n\t"
		"cmp    r0, #0                   \n\t"
		"ble    .DPRELOOPSUB_8x4             \n\t"
		"                                \n\t"
		"add    %3, %3, %11               \n\t"
		"add    %4, %4, %11               \n\t"
		"add    %5, %5, %11               \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DPRELOOPSUB_8x4:                   \n\t" // 
		"                                \n\t"
		"pld    [%3, #0]                \n\t"
		"pld    [%4, #0]                \n\t"
		"pld    [%5, #0]                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)
		"sub    %5, %5, #32              \n\t" // 
		"vld1.64   {d12, d13, d14, d15}, [%5:128]! \n\t" // load B to registers
		"vld1.64   {d4, d5, d6, d7},     [%5:128] \n\t" // load B to registers
		"vld1.64   {d8, d9, d10, d11},   [%3:128] \n\t" // load A0 to registers
		"vld1.64   {d24, d25},           [%4:128] \n\t" // load A1
		"pld    [%3, #64]                \n\t" // prefetch A0 to L1
		"pld    [%4, #64]                \n\t" // prefetch A1 to L1
		"pld    [%5, #32]                \n\t" // prefetch B to L1
#endif
#if defined(TARGET_CORTEX_A9)
		"pld    [%3, #32]                \n\t"
		"pld    [%4, #32]                \n\t"
		"pld    [%5, #32]                \n\t"
#endif
		"                                \n\t"
		"                                \n\t"
		"mov    r0, %2                   \n\t" // k_iter
		"                                \n\t"
		".DLOOPSUB_8x4:                      \n\t" // main loop 2
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)
		"                                \n\t"
		"vmls.f32  q0, q4, d12[0]        \n\t"
		"vldr   d26, [%4, #16]             \n\t"
		"vmls.f32  q1, q4, d12[1]        \n\t"
		"vldr   d27, [%4, #24]             \n\t"
		"vmls.f32  q14, q4, d13[0]        \n\t"
		"pld    [%3, #128]                \n\t"
		"vmls.f32  q15, q4, d13[1]        \n\t"
		"vldr   d8, [%3, #32]             \n\t"
		"                                \n\t"
		"vmls.f32  q8, q12, d12[0]        \n\t"
		"vldr   d9, [%3, #40]             \n\t"
		"vmls.f32  q9, q12, d12[1]        \n\t"
		"pld    [%4, #128]                \n\t"
		"vmls.f32  q10, q12, d13[0]        \n\t"
		"pld    [%5, #96]                \n\t"
		"vmls.f32  q11, q12, d13[1]        \n\t"
		"vldr   d24, [%4, #32]             \n\t"
		"                                \n\t"
		"vmls.f32  q0, q5, d14[0]        \n\t"
		"vldr   d25, [%4, #40]             \n\t"
		"vmls.f32  q1, q5, d14[1]        \n\t"
		"vldr   d12, [%5, #32]             \n\t"
		"vmls.f32  q14, q5, d15[0]        \n\t"
		"vldr   d13, [%5, #40]             \n\t"
		"vmls.f32  q15, q5, d15[1]        \n\t"
		"vldr   d10, [%3, #48]             \n\t"
		"                                \n\t"
		"vmls.f32  q8, q13, d14[0]        \n\t"
		"vldr   d11, [%3, #56]             \n\t"
		"vmls.f32  q9, q13, d14[1]        \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"vmls.f32  q10, q13, d15[0]        \n\t"
		"vldr   d14, [%5, #48]             \n\t"
		"vmls.f32  q11, q13, d15[1]        \n\t"
		"vldr   d26, [%4, #48]             \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmls.f32  q0, q4, d4[0]        \n\t"
		"vldr   d27, [%4, #56]             \n\t"
		"vmls.f32  q1, q4, d4[1]        \n\t"
		"vldr   d15, [%5, #56]             \n\t"
		"vmls.f32  q14, q4, d5[0]        \n\t"
		"cmp    r0, #0                   \n\t" // next iter?
		"vmls.f32  q15, q4, d5[1]        \n\t"
		"vldr   d8, [%3, #64]             \n\t"
		"                                \n\t"
		"vmls.f32  q8, q12, d4[0]        \n\t"
		"vldr   d9, [%3, #72]             \n\t"
		"vmls.f32  q9, q12, d4[1]        \n\t"
		"vmls.f32  q10, q12, d5[0]        \n\t"
		"vldr   d4, [%5, #64]             \n\t"
		"vmls.f32  q11, q12, d5[1]        \n\t"
		"vldr   d24, [%4, #64]             \n\t"
		"                                \n\t"
		"vmls.f32  q0, q5, d6[0]        \n\t"
		"vldr   d25, [%4, #72]             \n\t"
		"vmls.f32  q1, q5, d6[1]        \n\t"
		"vldr   d5, [%5, #72]             \n\t"
		"vmls.f32  q14, q5, d7[0]        \n\t"
		"add    %4, %4, #64              \n\t" // increase A
		"vmls.f32  q15, q5, d7[1]        \n\t"
		"vldr   d10, [%3, #80]             \n\t"
		"                                \n\t"
		"vmls.f32  q8, q13, d6[0]        \n\t"
		"vldr   d11, [%3, #88]             \n\t"
		"vmls.f32  q9, q13, d6[1]        \n\t"
		"add    %3, %3, #64              \n\t" // increase A
		"vmls.f32  q10, q13, d7[0]        \n\t"
		"vldr   d6, [%5, #80]             \n\t"
		"vmls.f32  q11, q13, d7[1]        \n\t"
		"vldr   d7, [%5, #88]             \n\t"
		"add    %5, %5, #64              \n\t" // increase A
		"                                \n\t"
#endif
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)
		"                                \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%5:128]! \n\t" // load B
		"vld1.64   {d8, d9, d10, d11},   [%3:128]! \n\t" // load A0
		"vld1.64   {d24, d25, d26, d27}, [%4:128]! \n\t" // load A0
		"                                \n\t"
		"pld    [%3, #32]                \n\t"
		"pld    [%4, #32]                \n\t"
		"pld    [%5, #32]                \n\t"
		"                                \n\t"
		"vmls.f32  q0, q4, d12[0]        \n\t"
		"vmls.f32  q1, q4, d12[1]        \n\t"
		"vmls.f32  q14, q4, d13[0]        \n\t"
		"vmls.f32  q15, q4, d13[1]        \n\t"
		"                                \n\t"
		"vmls.f32  q8, q12, d12[0]        \n\t"
		"vmls.f32  q9, q12, d12[1]        \n\t"
		"vmls.f32  q10, q12, d13[0]        \n\t"
		"vmls.f32  q11, q12, d13[1]        \n\t"
		"                                \n\t"
		"vmls.f32  q0, q5, d14[0]        \n\t"
		"vmls.f32  q1, q5, d14[1]        \n\t"
		"vmls.f32  q14, q5, d15[0]        \n\t"
		"vmls.f32  q15, q5, d15[1]        \n\t"
		"                                \n\t"
		"vmls.f32  q8, q13, d14[0]        \n\t"
		"vmls.f32  q9, q13, d14[1]        \n\t"
		"vmls.f32  q10, q13, d15[0]        \n\t"
		"vmls.f32  q11, q13, d15[1]        \n\t"
		"                                \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%5:128]! \n\t" // load B
		"vld1.64   {d8, d9, d10, d11},   [%3:128]! \n\t" // load A0
		"vld1.64   {d24, d25, d26, d27}, [%4:128]! \n\t" // load A0
		"                                \n\t"
#if defined(TARGET_CORTEX_A9)
		"pld    [%3, #32]                \n\t"
		"pld    [%4, #32]                \n\t"
		"pld    [%5, #32]                \n\t"
#endif
		"                                \n\t"
		"cmp    r0, #0                   \n\t" // next iter?
		"                                \n\t"
		"vmls.f32  q0, q4, d12[0]        \n\t"
		"vmls.f32  q1, q4, d12[1]        \n\t"
		"vmls.f32  q14, q4, d13[0]        \n\t"
		"vmls.f32  q15, q4, d13[1]        \n\t"
		"                                \n\t"
		"vmls.f32  q8, q12, d12[0]        \n\t"
		"vmls.f32  q9, q12, d12[1]        \n\t"
		"vmls.f32  q10, q12, d13[0]        \n\t"
		"vmls.f32  q11, q12, d13[1]        \n\t"
		"                                \n\t"
		"vmls.f32  q0, q5, d14[0]        \n\t"
		"vmls.f32  q1, q5, d14[1]        \n\t"
		"vmls.f32  q14, q5, d15[0]        \n\t"
		"vmls.f32  q15, q5, d15[1]        \n\t"
		"                                \n\t"
		"vmls.f32  q8, q13, d14[0]        \n\t"
		"vmls.f32  q9, q13, d14[1]        \n\t"
		"vmls.f32  q10, q13, d15[0]        \n\t"
		"vmls.f32  q11, q13, d15[1]        \n\t"
		"                                \n\t"
		"                                \n\t"
#endif
		"                                \n\t"
		"bgt    .DLOOPSUB_8x4            \n\t"
		"                                \n\t"
		"                                \n\t"
		".DPOSTACC_8x4:                  \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmov      q2, q14               \n\t"
		"vmov      q3, q15               \n\t"
		"                                \n\t"
		"                                \n\t"
		"vld1.64   {d8, d9, d10, d11},   [%6:128]! \n\t" // load C0
		"vld1.64   {d12, d13, d14, d15}, [%6:128]  \n\t" // load C0
		"vld1.64   {d24, d25, d26, d27}, [%7:128]! \n\t" // load C1
		"vld1.64   {d28, d29, d30, d31}, [%7:128]  \n\t" // load C1
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vadd.f32  q4, q0, q4            \n\t"
		"vadd.f32  q5, q1, q5            \n\t"
		"vadd.f32  q6, q2, q6            \n\t"
		"vadd.f32  q7, q3, q7            \n\t"
		"                                \n\t"
		"vadd.f32  q8,  q8,  q12         \n\t"
		"vadd.f32  q9,  q9,  q13         \n\t"
		"vadd.f32  q10, q10, q14         \n\t"
		"vadd.f32  q11, q11, q15         \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vld1.64   {d0, d1, d2, d3},   [%10]! \n\t" // fact
		"vld1.64   {d4},   [%10]! \n\t" // fact
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmul.f32  q4, q4, d0[0]         \n\t"
		"vmul.f32  q8, q8, d0[0]         \n\t"
		"                                \n\t"
		"vst1.64   {d8, d9},   [%8:128]! \n\t" // store D0
		"vst1.64   {d16, d17}, [%9:128]! \n\t" // store D1
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmls.f32  q5, q4, d0[1]         \n\t"
		"vmls.f32  q9, q8, d0[1]         \n\t"
		"                                \n\t"
		"vmul.f32  q5, q5, d1[0]         \n\t"
		"vmul.f32  q9, q9, d1[0]         \n\t"
		"                                \n\t"
		"vst1.64   {d10, d11}, [%8:128]! \n\t" // store D0
		"vst1.64   {d18, d19}, [%9:128]! \n\t" // store D1
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmls.f32  q6, q4, d1[1]         \n\t"
		"vmls.f32  q10, q8, d1[1]         \n\t"
		"                                \n\t"
		"vmls.f32  q6, q5, d2[0]         \n\t"
		"vmls.f32  q10, q9, d2[0]         \n\t"
		"                                \n\t"
		"vmul.f32  q6, q6, d2[1]         \n\t"
		"vmul.f32  q10, q10, d2[1]       \n\t"
		"                                \n\t"
		"vst1.64   {d12, d13}, [%8:128]! \n\t" // store D0
		"vst1.64   {d20, d21}, [%9:128]! \n\t" // store D1
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmls.f32  q7, q4, d3[0]         \n\t"
		"vmls.f32  q11, q8, d3[0]         \n\t"
		"                                \n\t"
		"vmls.f32  q7, q5, d3[1]         \n\t"
		"vmls.f32  q11, q9, d3[1]         \n\t"
		"                                \n\t"
		"vmls.f32  q7, q6, d4[0]         \n\t"
		"vmls.f32  q11, q10, d4[0]         \n\t"
		"                                \n\t"
		"vmul.f32  q7, q7, d4[1]         \n\t"
		"vmul.f32  q11, q11, d4[1]         \n\t"
		"                                \n\t"
		"vst1.64   {d14, d15}, [%8:128]! \n\t" // store D0
		"vst1.64   {d22, d23}, [%9:128]! \n\t" // store D1
		"                                \n\t"
		"                                \n\t"
		".DEND_8x4:                      \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		: // output operands (none)
		: // input operands
		  "r" (ki_add),		// %0
		  "r" (kl_add),		// %1
		  "r" (ki_sub),		// %2
		  "r" (A0),			// %3
		  "r" (A1),			// %4
		  "r" (B),			// %5
		  "r" (C0),			// %6
		  "r" (C1),			// %7
		  "r" (D0),			// %8
		  "r" (D1),			// %9
		  "r" (fact),		// %10
		  "r" (dA)			// %11
		: // register clobber list
		  "r0",
		  "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",
		  "d8", "d9", "d10", "d11", "d12", "d13", "d14", "d15",
		  "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23",
		  "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31",
		  "memory"
	);
}



void kernel_sgemm_strsm_nt_4x4_lib4(int kadd, int ksub, float *A0, float *B, float *C0, float *D0, float *fact)
	{

/*	if(kmax<=0)*/
/*		return;*/
		
	__builtin_prefetch( A0 );
	__builtin_prefetch( B  );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A0+8 );
	__builtin_prefetch( B +8 );
#endif

	int ki_add = kadd/4;
	int kl_add = kadd%4;
	int ki_sub = ksub/4;

	const int bs = S_MR;//4;
	const int d_ncl = S_NCL;//2;

	int dA = bs*((d_ncl-kadd%d_ncl)%d_ncl)*sizeof(float);
	
/*	printf("\n%d %d %d\n", kadd, ksub, dA);*/

	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)
		"pld    [%3, #64]                \n\t" // prefetch A0 to L1
		"pld    [%4, #64]                \n\t" // prefetch B to L1
#endif
		"                                \n\t"
		"                                \n\t"
		"mov    r0, %0                   \n\t" // k_iter
		"                                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)
		"vld1.64   {d12, d13, d14, d15}, [%4:128]! \n\t" // load B to registers
		"vld1.64   {d4, d5, d6, d7},     [%4:128] \n\t" // load B to registers
/*		"vld1.64   {d8, d9, d10, d11},   [%3:128] \n\t" // load A0 to registers*/
		"vld1.64   {d8, d9},   [%3:128] \n\t" // load A0 to registers
#endif
		"                                \n\t"
		"cmp    r0, #0                   \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vldr   d0, .DZERO_4x4               \n\t" // load zero double
		"vldr   d1, .DZERO_4x4+8             \n\t" // load zero double
		"vmov   q1, q0                   \n\t"
		"vmov   q14, q0                   \n\t"
		"vmov   q15, q0                   \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"ble    .DCONSIDERADD2_4x4           \n\t"
		"                                \n\t"
		"                                \n\t"
		"b      .DENDZERO_4x4                \n\t"
		".align 3                        \n\t"
		".DZERO_4x4:                         \n\t" // zero quad word
		".word  0                        \n\t"
		".word  0                        \n\t"
		".word  0                        \n\t"
		".word  0                        \n\t"
		".DENDZERO_4x4:                      \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DLOOPADD_4x4:                    \n\t" // main loop
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)
		"                                \n\t"
		"vmla.f32  q0, q4, d12[0]        \n\t"
		"vldr   d10, [%3, #16]             \n\t"
		"vmla.f32  q1, q4, d12[1]        \n\t"
		"vldr   d11, [%3, #24]             \n\t"
		"pld    [%4, #96]                \n\t"
		"vmla.f32  q14, q4, d13[0]        \n\t"
		"pld    [%3, #128]                \n\t"
		"vmla.f32  q15, q4, d13[1]        \n\t"
		"vldr   d8, [%3, #32]             \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q0, q5, d14[0]        \n\t"
		"vldr   d9, [%3, #40]             \n\t"
		"vmla.f32  q1, q5, d14[1]        \n\t"
		"vldr   d12, [%4, #32]             \n\t"
		"vmla.f32  q14, q5, d15[0]        \n\t"
		"vldr   d13, [%4, #40]             \n\t"
		"vmla.f32  q15, q5, d15[1]        \n\t"
		"vldr   d10, [%3, #48]             \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"vldr   d11, [%3, #56]             \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q0, q4, d4[0]        \n\t"
		"vldr   d14, [%4, #48]             \n\t"
		"vmla.f32  q1, q4, d4[1]        \n\t"
		"vldr   d15, [%4, #56]             \n\t"
		"vmla.f32  q14, q4, d5[0]        \n\t"
		"cmp    r0, #0                   \n\t" // next iter?
		"vmla.f32  q15, q4, d5[1]        \n\t"
		"vldr   d8, [%3, #64]             \n\t"
		"add    %4, %4, #64              \n\t" // increase A
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q0, q5, d6[0]        \n\t"
		"vldr   d9, [%3, #72]             \n\t"
		"vmla.f32  q1, q5, d6[1]        \n\t"
		"vldr   d4, [%4, #0]             \n\t"
		"vmla.f32  q14, q5, d7[0]        \n\t"
		"vldr   d5, [%4, #8]             \n\t"
		"add    %3, %3, #64              \n\t" // increase A
		"vmla.f32  q15, q5, d7[1]        \n\t"
		"vldr   d6, [%4, #16]             \n\t"
		"vldr   d7, [%4, #24]             \n\t"
		"                                \n\t"
#endif
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)
		"                                \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%4:128]! \n\t" // load B
		"vld1.64   {d8, d9, d10, d11},   [%3:128]! \n\t" // load A0
		"                                \n\t"
		"pld    [%3, #32]                \n\t"
		"pld    [%4, #32]                \n\t"
		"                                \n\t"
		"vmla.f32  q0, q4, d12[0]        \n\t"
		"vmla.f32  q1, q4, d12[1]        \n\t"
		"vmla.f32  q14, q4, d13[0]        \n\t"
		"vmla.f32  q15, q4, d13[1]        \n\t"
		"                                \n\t"
		"vmla.f32  q0, q5, d14[0]        \n\t"
		"vmla.f32  q1, q5, d14[1]        \n\t"
		"vmla.f32  q14, q5, d15[0]        \n\t"
		"vmla.f32  q15, q5, d15[1]        \n\t"
		"                                \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%4:128]! \n\t" // load B
		"vld1.64   {d8, d9, d10, d11},   [%3:128]! \n\t" // load A0
		"                                \n\t"
#if defined(TARGET_CORTEX_A9)
		"pld    [%3, #32]                \n\t"
		"pld    [%4, #32]                \n\t"
#endif
		"                                \n\t"
		"cmp    r0, #0                   \n\t" // next iter?
		"                                \n\t"
		"vmla.f32  q0, q4, d12[0]        \n\t"
		"vmla.f32  q1, q4, d12[1]        \n\t"
		"vmla.f32  q14, q4, d13[0]        \n\t"
		"vmla.f32  q15, q4, d13[1]        \n\t"
		"                                \n\t"
		"vmla.f32  q0, q5, d14[0]        \n\t"
		"vmla.f32  q1, q5, d14[1]        \n\t"
		"vmla.f32  q14, q5, d15[0]        \n\t"
		"vmla.f32  q15, q5, d15[1]        \n\t"
		"                                \n\t"
		"                                \n\t"
#endif
		"                                \n\t"
		"bgt    .DLOOPADD_4x4              \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERADD2_4x4:                 \n\t" // consider left k+=2
		"                                \n\t"
		"mov    r0, %1                   \n\t" // k_left
		"cmp    r0, #1                   \n\t"
		"ble    .DCONSIDERADD1_4x4          \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)
		"                                \n\t"
		"vmla.f32  q0, q4, d12[0]        \n\t"
		"vldr   d10, [%3, #16]             \n\t"
		"vmla.f32  q1, q4, d12[1]        \n\t"
		"vldr   d11, [%3, #24]             \n\t"
		"vmla.f32  q14, q4, d13[0]        \n\t"
		"vmla.f32  q15, q4, d13[1]        \n\t"
		"vldr   d8, [%3, #32]             \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q0, q5, d14[0]        \n\t"
		"vldr   d9, [%3, #40]             \n\t"
		"vmla.f32  q1, q5, d14[1]        \n\t"
//		"vldr   d12, [%4, #32]             \n\t"
		"add    %3, %3, #32              \n\t" // increase A
		"vmov   d12, d4                  \n\t"
		"vmla.f32  q14, q5, d15[0]        \n\t"
//		"vldr   d13, [%4, #40]             \n\t"
		"add    %4, %4, #32              \n\t" // increase A
		"vmov   d13, d5                  \n\t"
		"vmla.f32  q15, q5, d15[1]        \n\t"
		"                                \n\t"
		"                                \n\t"
#endif
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)
		"                                \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%4:128]! \n\t" // load B
		"vld1.64   {d8, d9, d10, d11},   [%3:128]! \n\t" // load A0
		"                                \n\t"
		"vmla.f32  q0, q4, d12[0]        \n\t"
		"vmla.f32  q1, q4, d12[1]        \n\t"
		"vmla.f32  q14, q4, d13[0]        \n\t"
		"vmla.f32  q15, q4, d13[1]        \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q0, q5, d14[0]        \n\t"
		"vmla.f32  q1, q5, d14[1]        \n\t"
		"vmla.f32  q14, q5, d15[0]        \n\t"
		"vmla.f32  q15, q5, d15[1]        \n\t"
		"                                \n\t"
		"                                \n\t"
#endif
		"                                \n\t"
		"sub    r0, r0, #2               \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERADD1_4x4:                 \n\t" // consider left k++
		"                                \n\t"
		"cmp    r0, #0                   \n\t"
		"ble    .DCONSIDERSUB_4x4              \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)
		"                                \n\t"
		"vld1.64   {d12, d13}, [%4:128]! \n\t" // load B
		"vld1.64   {d8, d9},   [%3:128]! \n\t" // load A0
		"                                \n\t"
		"vmla.f32  q0, q4, d12[0]        \n\t"
		"vmla.f32  q1, q4, d12[1]        \n\t"
		"vmla.f32  q14, q4, d13[0]        \n\t"
		"vmla.f32  q15, q4, d13[1]        \n\t"
		"                                \n\t"
		"                                \n\t"
#endif
#if defined(TARGET_CORTEX_A15)
		"                                \n\t"
		"vmla.f32  q0, q4, d12[0]        \n\t"
		"vmla.f32  q1, q4, d12[1]        \n\t"
		"add    %3, %3, #16              \n\t" // increase A
		"vmla.f32  q14, q4, d13[0]        \n\t"
		"add    %4, %4, #16              \n\t" // increase A
		"vmla.f32  q15, q4, d13[1]        \n\t"
		"                                \n\t"
		"                                \n\t"
#endif
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERSUB_4x4:                    \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"add    r0, %0, %1               \n\t"
		"                                \n\t"
		"cmp    %2, #0                   \n\t"
		"ble    .DPOSTACC_4x4                \n\t"
		"                                \n\t"
		"cmp    %8, #0                   \n\t"
		"ble    .DPRELOOPSUB_4x4             \n\t"
		"                                \n\t"
		"cmp    r0, #0                   \n\t"
		"ble    .DPRELOOPSUB_4x4             \n\t"
		"                                \n\t"
		"add    %3, %3, %8               \n\t"
		"add    %4, %4, %8               \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DPRELOOPSUB_4x4:                   \n\t" // 
		"                                \n\t"
		"pld    [%3, #0]                \n\t"
		"pld    [%4, #0]                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)
		"sub    %4, %4, #32              \n\t" // 
		"vld1.64   {d12, d13, d14, d15}, [%4:128]! \n\t" // load B to registers
		"vld1.64   {d4, d5, d6, d7},     [%4:128] \n\t" // load B to registers
/*		"vld1.64   {d8, d9, d10, d11},   [%3:128] \n\t" // load A0 to registers*/
		"vld1.64   {d8, d9},   [%3:128] \n\t" // load A0 to registers
		"pld    [%3, #64]                \n\t" // prefetch A0 to L1
		"pld    [%4, #32]                \n\t" // prefetch B to L1
#endif
#if defined(TARGET_CORTEX_A9)
		"pld    [%3, #32]                \n\t"
		"pld    [%4, #32]                \n\t"
#endif
		"                                \n\t"
		"                                \n\t"
		"mov    r0, %2                   \n\t" // k_iter
		"                                \n\t"
		".DLOOPSUB_4x4:                      \n\t" // main loop 2
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)
		"                                \n\t"
		"vmls.f32  q0, q4, d12[0]        \n\t"
		"vldr   d10, [%3, #16]             \n\t"
		"vmls.f32  q1, q4, d12[1]        \n\t"
		"vldr   d11, [%3, #24]             \n\t"
		"vmls.f32  q14, q4, d13[0]        \n\t"
		"pld    [%3, #128]                \n\t"
		"vmls.f32  q15, q4, d13[1]        \n\t"
		"vldr   d8, [%3, #32]             \n\t"
		"pld    [%4, #96]                \n\t"
		"                                \n\t"
		"vmls.f32  q0, q5, d14[0]        \n\t"
		"vldr   d9, [%3, #40]             \n\t"
		"vmls.f32  q1, q5, d14[1]        \n\t"
		"vldr   d12, [%4, #32]             \n\t"
		"vmls.f32  q14, q5, d15[0]        \n\t"
		"vldr   d13, [%4, #40]             \n\t"
		"vmls.f32  q15, q5, d15[1]        \n\t"
		"vldr   d10, [%3, #48]             \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"vldr   d11, [%3, #56]             \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmls.f32  q0, q4, d4[0]        \n\t"
		"vldr   d14, [%4, #48]             \n\t"
		"vmls.f32  q1, q4, d4[1]        \n\t"
		"vldr   d15, [%4, #56]             \n\t"
		"vmls.f32  q14, q4, d5[0]        \n\t"
		"cmp    r0, #0                   \n\t" // next iter?
		"vmls.f32  q15, q4, d5[1]        \n\t"
		"vldr   d8, [%3, #64]             \n\t"
		"add    %4, %4, #64              \n\t" // increase A
		"                                \n\t"
		"vmls.f32  q0, q5, d6[0]        \n\t"
		"vldr   d9, [%3, #72]             \n\t"
		"vmls.f32  q1, q5, d6[1]        \n\t"
		"vldr   d4, [%4, #0]             \n\t"
		"vmls.f32  q14, q5, d7[0]        \n\t"
		"vldr   d5, [%4, #8]             \n\t"
		"add    %3, %3, #64              \n\t" // increase A
		"vmls.f32  q15, q5, d7[1]        \n\t"
		"vldr   d6, [%4, #16]             \n\t"
		"vldr   d7, [%4, #24]             \n\t"
		"                                \n\t"
#endif
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)
		"                                \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%4:128]! \n\t" // load B
		"vld1.64   {d8, d9, d10, d11},   [%3:128]! \n\t" // load A0
		"                                \n\t"
		"pld    [%3, #32]                \n\t"
		"pld    [%4, #32]                \n\t"
		"                                \n\t"
		"vmls.f32  q0, q4, d12[0]        \n\t"
		"vmls.f32  q1, q4, d12[1]        \n\t"
		"vmls.f32  q14, q4, d13[0]        \n\t"
		"vmls.f32  q15, q4, d13[1]        \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmls.f32  q0, q5, d14[0]        \n\t"
		"vmls.f32  q1, q5, d14[1]        \n\t"
		"vmls.f32  q14, q5, d15[0]        \n\t"
		"vmls.f32  q15, q5, d15[1]        \n\t"
		"                                \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%4:128]! \n\t" // load B
		"vld1.64   {d8, d9, d10, d11},   [%3:128]! \n\t" // load A0
		"                                \n\t"
#if defined(TARGET_CORTEX_A9)
		"pld    [%3, #32]                \n\t"
		"pld    [%4, #32]                \n\t"
#endif
		"                                \n\t"
		"cmp    r0, #0                   \n\t" // next iter?
		"                                \n\t"
		"vmls.f32  q0, q4, d12[0]        \n\t"
		"vmls.f32  q1, q4, d12[1]        \n\t"
		"vmls.f32  q14, q4, d13[0]        \n\t"
		"vmls.f32  q15, q4, d13[1]        \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmls.f32  q0, q5, d14[0]        \n\t"
		"vmls.f32  q1, q5, d14[1]        \n\t"
		"vmls.f32  q14, q5, d15[0]        \n\t"
		"vmls.f32  q15, q5, d15[1]        \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
#endif
		"                                \n\t"
		"bgt    .DLOOPSUB_4x4            \n\t"
		"                                \n\t"
		"                                \n\t"
		".DPOSTACC_4x4:                  \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmov      q2, q14               \n\t"
		"vmov      q3, q15               \n\t"
		"                                \n\t"
		"                                \n\t"
		"vld1.64   {d8, d9, d10, d11},   [%5:128]! \n\t" // load C0
		"vld1.64   {d12, d13, d14, d15}, [%5:128]  \n\t" // load C0
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vadd.f32  q4, q0, q4            \n\t"
		"vadd.f32  q5, q1, q5            \n\t"
		"vadd.f32  q6, q2, q6            \n\t"
		"vadd.f32  q7, q3, q7            \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vld1.64   {d0, d1, d2, d3},   [%7]! \n\t" // fact
		"vld1.64   {d4},   [%7]! \n\t" // fact
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmul.f32  q4, q4, d0[0]         \n\t"
		"                                \n\t"
		"vst1.64   {d8, d9},   [%6:128]! \n\t" // store D0
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmls.f32  q5, q4, d0[1]         \n\t"
		"                                \n\t"
		"vmul.f32  q5, q5, d1[0]         \n\t"
		"                                \n\t"
		"vst1.64   {d10, d11}, [%6:128]! \n\t" // store D0
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmls.f32  q6, q4, d1[1]         \n\t"
		"                                \n\t"
		"vmls.f32  q6, q5, d2[0]         \n\t"
		"                                \n\t"
		"vmul.f32  q6, q6, d2[1]         \n\t"
		"                                \n\t"
		"vst1.64   {d12, d13}, [%6:128]! \n\t" // store D0
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmls.f32  q7, q4, d3[0]         \n\t"
		"                                \n\t"
		"vmls.f32  q7, q5, d3[1]         \n\t"
		"                                \n\t"
		"vmls.f32  q7, q6, d4[0]         \n\t"
		"                                \n\t"
		"vmul.f32  q7, q7, d4[1]         \n\t"
		"                                \n\t"
		"vst1.64   {d14, d15}, [%6:128]! \n\t" // store D0
		"                                \n\t"
		"                                \n\t"
		".DEND_4x4:                      \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		: // output operands (none)
		: // input operands
		  "r" (ki_add),		// %0
		  "r" (kl_add),		// %1
		  "r" (ki_sub),		// %2
		  "r" (A0),			// %3
		  "r" (B),			// %4
		  "r" (C0),			// %5
		  "r" (D0),			// %6
		  "r" (fact),		// %7
		  "r" (dA)			// %8
		: // register clobber list
		  "r0",
		  "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",
		  "d8", "d9", "d10", "d11", "d12", "d13", "d14", "d15",
		  "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23",
		  "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31",
		  "memory"
	);
}

	
	
void kernel_sgemm_strsm_nt_4x2_lib4(int kadd, int ksub, float *A, float *B, float *C, float *D, float *fact)
	{

	const int bs = 4;
	const int d_ncl = S_NCL;
	const int lda = bs;
	const int ldc = bs;

	int k;

	float
		a_0, a_1, a_2, a_3,
		b_0, b_1,
		c_00=0, c_01=0,
		c_10=0, c_11=0,
		c_20=0, c_21=0,
		c_30=0, c_31=0;
		
	for(k=0; k<kadd-3; k+=4)
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
	for(; k<kadd; k++)
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

	if(ksub>0)
		{
		if(kadd>0)
			{
			A += bs*((d_ncl-kadd%d_ncl)%d_ncl);
			B += bs*((d_ncl-kadd%d_ncl)%d_ncl);
			}
		}

	for(k=0; k<ksub-3; k+=4)
		{
		
		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		a_2 = A[2+lda*0];
		a_3 = A[3+lda*0];
		
		b_0 = B[0+lda*0];
		b_1 = B[1+lda*0];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;


		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		a_2 = A[2+lda*1];
		a_3 = A[3+lda*1];
		
		b_0 = B[0+lda*1];
		b_1 = B[1+lda*1];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;


		a_0 = A[0+lda*2];
		a_1 = A[1+lda*2];
		a_2 = A[2+lda*2];
		a_3 = A[3+lda*2];
		
		b_0 = B[0+lda*2];
		b_1 = B[1+lda*2];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;


		a_0 = A[0+lda*3];
		a_1 = A[1+lda*3];
		a_2 = A[2+lda*3];
		a_3 = A[3+lda*3];
		
		b_0 = B[0+lda*3];
		b_1 = B[1+lda*3];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;
		
		
		A += 16;
		B += 16;

		}

	c_00 += C[0+ldc*0];
	c_10 += C[1+ldc*0];
	c_20 += C[2+ldc*0];
	c_30 += C[3+ldc*0];

	c_01 += C[0+ldc*1];
	c_11 += C[1+ldc*1];
	c_21 += C[2+ldc*1];
	c_31 += C[3+ldc*1];
	
	// strsm
	float
		a_00, a_10, a_11;
	
	a_00 = fact[0];
	c_00 *= a_00;
	c_10 *= a_00;
	c_20 *= a_00;
	c_30 *= a_00;
	D[0+ldc*0] = c_00;
	D[1+ldc*0] = c_10;
	D[2+ldc*0] = c_20;
	D[3+ldc*0] = c_30;

	a_10 = fact[1];
	a_11 = fact[2];
	c_01 -= c_00*a_10;
	c_11 -= c_10*a_10;
	c_21 -= c_20*a_10;
	c_31 -= c_30*a_10;
	c_01 *= a_11;
	c_11 *= a_11;
	c_21 *= a_11;
	c_31 *= a_11;
	D[0+ldc*1] = c_01;
	D[1+ldc*1] = c_11;
	D[2+ldc*1] = c_21;
	D[3+ldc*1] = c_31;

	}
	
	
	
void kernel_sgemm_strsm_nt_2x4_lib4(int kadd, int ksub, float *A, float *B, float *C, float *D, float *fact)
	{

	const int bs = 4;
	const int d_ncl = S_NCL;
	const int lda = bs;
	const int ldc = bs;

	int k;

	float
		a_0, a_1,
		b_0, b_1, b_2, b_3,
		c_00=0, c_01=0, c_02=0, c_03=0,
		c_10=0, c_11=0, c_12=0, c_13=0;
		
	for(k=0; k<kadd-3; k+=4)
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
	for(; k<kadd; k++)
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

	if(ksub>0)
		{
		if(kadd>0)
			{
			A += bs*((d_ncl-kadd%d_ncl)%d_ncl);
			B += bs*((d_ncl-kadd%d_ncl)%d_ncl);
			}
		}

	for(k=0; k<ksub-3; k+=4)
		{
		
		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		
		b_0 = B[0+lda*0];
		b_1 = B[1+lda*0];
		b_2 = B[2+lda*0];
		b_3 = B[3+lda*0];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;

		c_02 -= a_0 * b_2;
		c_12 -= a_1 * b_2;

		c_03 -= a_0 * b_3;
		c_13 -= a_1 * b_3;


		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		
		b_0 = B[0+lda*1];
		b_1 = B[1+lda*1];
		b_2 = B[2+lda*1];
		b_3 = B[3+lda*1];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;

		c_02 -= a_0 * b_2;
		c_12 -= a_1 * b_2;

		c_03 -= a_0 * b_3;
		c_13 -= a_1 * b_3;


		a_0 = A[0+lda*2];
		a_1 = A[1+lda*2];
		
		b_0 = B[0+lda*2];
		b_1 = B[1+lda*2];
		b_2 = B[2+lda*2];
		b_3 = B[3+lda*2];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;

		c_02 -= a_0 * b_2;
		c_12 -= a_1 * b_2;

		c_03 -= a_0 * b_3;
		c_13 -= a_1 * b_3;


		a_0 = A[0+lda*3];
		a_1 = A[1+lda*3];
		
		b_0 = B[0+lda*3];
		b_1 = B[1+lda*3];
		b_2 = B[2+lda*3];
		b_3 = B[3+lda*3];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;

		c_02 -= a_0 * b_2;
		c_12 -= a_1 * b_2;

		c_03 -= a_0 * b_3;
		c_13 -= a_1 * b_3;
		
		
		A += 16;
		B += 16;

		}

	c_00 += C[0+ldc*0];
	c_10 += C[1+ldc*0];

	c_01 += C[0+ldc*1];
	c_11 += C[1+ldc*1];

	c_02 += C[0+ldc*2];
	c_12 += C[1+ldc*2];

	c_03 += C[0+ldc*3];
	c_13 += C[1+ldc*3];
	
	// strsm
	float
		a_00, a_10, a_20, a_30, a_11, a_21, a_31, a_22, a_32, a_33;
	
	a_00 = fact[0];
	c_00 *= a_00;
	c_10 *= a_00;
	D[0+ldc*0] = c_00;
	D[1+ldc*0] = c_10;

	a_10 = fact[1];
	a_11 = fact[2];
	c_01 -= c_00*a_10;
	c_11 -= c_10*a_10;
	c_01 *= a_11;
	c_11 *= a_11;
	D[0+ldc*1] = c_01;
	D[1+ldc*1] = c_11;

	a_20 = fact[3];
	a_21 = fact[4];
	a_22 = fact[5];
	c_02 -= c_00*a_20;
	c_12 -= c_10*a_20;
	c_02 -= c_01*a_21;
	c_12 -= c_11*a_21;
	c_02 *= a_22;
	c_12 *= a_22;
	D[0+ldc*2] = c_02;
	D[1+ldc*2] = c_12;

	a_30 = fact[6];
	a_31 = fact[7];
	a_32 = fact[8];
	a_33 = fact[9];
	c_03 -= c_00*a_30;
	c_13 -= c_10*a_30;
	c_03 -= c_01*a_31;
	c_13 -= c_11*a_31;
	c_03 -= c_02*a_32;
	c_13 -= c_12*a_32;
	c_03 *= a_33;
	c_13 *= a_33;
	D[0+ldc*3] = c_03;
	D[1+ldc*3] = c_13;

	}
	
	
	
void kernel_sgemm_strsm_nt_2x2_lib4(int kadd, int ksub, float *A, float *B, float *C, float *D, float *fact)
	{

	const int bs = 4;
	const int d_ncl = S_NCL;
	const int lda = bs;
	const int ldc = bs;

	int k;

	float
		a_0, a_1,
		b_0, b_1,
		c_00=0, c_01=0,
		c_10=0, c_11=0;
		
	for(k=0; k<kadd-3; k+=4)
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
	for(; k<kadd; k++)
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

	if(ksub>0)
		{
		if(kadd>0)
			{
			A += bs*((d_ncl-kadd%d_ncl)%d_ncl);
			B += bs*((d_ncl-kadd%d_ncl)%d_ncl);
			}
		}

	for(k=0; k<ksub-3; k+=4)
		{
		
		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		
		b_0 = B[0+lda*0];
		b_1 = B[1+lda*0];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;


		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		
		b_0 = B[0+lda*1];
		b_1 = B[1+lda*1];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;


		a_0 = A[0+lda*2];
		a_1 = A[1+lda*2];
		
		b_0 = B[0+lda*2];
		b_1 = B[1+lda*2];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;


		a_0 = A[0+lda*3];
		a_1 = A[1+lda*3];
		
		b_0 = B[0+lda*3];
		b_1 = B[1+lda*3];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_01 -= a_0 * b_1;
		c_11 -= a_1 * b_1;
		
		
		A += 16;
		B += 16;

		}

	c_00 += C[0+ldc*0];
	c_10 += C[1+ldc*0];

	c_01 += C[0+ldc*1];
	c_11 += C[1+ldc*1];
	
	// strsm
	float
		a_00, a_10, a_11;
	
	a_00 = fact[0];
	c_00 *= a_00;
	c_10 *= a_00;
	D[0+ldc*0] = c_00;
	D[1+ldc*0] = c_10;

	a_10 = fact[1];
	a_11 = fact[2];
	c_01 -= c_00*a_10;
	c_11 -= c_10*a_10;
	c_01 *= a_11;
	c_11 *= a_11;
	D[0+ldc*1] = c_01;
	D[1+ldc*1] = c_11;

	}
	
	
	

