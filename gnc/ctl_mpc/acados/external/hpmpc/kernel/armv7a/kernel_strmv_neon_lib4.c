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



/*void kernel_sgemv_t_8_lib4(int kmax, int kna, float *A, int sda, float *x, float *y, int alg)*/
void kernel_strmv_u_t_8_lib4(int kmax, float *A, int sda, float *x, float *y, int alg)
	{

/*	if(kmax<=0) */
/*		return;*/
	
	const int bs = 4;

	float *A0 = A;
	float *x0 = x;
	
	__builtin_prefetch( A0 );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A0+8 );
#endif		

	int incA = bs*(sda-8)*sizeof(float);

	int k_iter = kmax/4;
	int k_left = kmax%4;

	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"pld    [%1, #64]                \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%1, #96]                \n\t" // prefetch A1 to L1
#endif		
		"                                \n\t"
		"                                \n\t"
		"add    r1, %5, #64              \n\t"
#if defined(TARGET_CORTEX_A9)
		"add    r2, r1, #32              \n\t"
#endif		
		"                                \n\t"
		"                                \n\t"
		"vldr   d16, .DZERO_T_8          \n\t" // load zero double
		"vldr   d17, .DZERO_T_8+8        \n\t" // load zero double
		"vmov   q9, q8                   \n\t"
		"vmov   q10, q8                  \n\t"
		"vmov   q11, q8                  \n\t"
		"vmov   q12, q8                  \n\t"
		"vmov   q13, q8                  \n\t"
		"vmov   q14, q8                  \n\t"
		"vmov   q15, q8                  \n\t"
		"vmov   q2, q8                   \n\t" // zero vector
		"                                \n\t"
		"                                \n\t"
		"vld1.64   {d0, d1, d2, d3}, [%2:128]!   \n\t" // load x to registers
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13, d14, d15}, [%1:128]!   \n\t" // load A0 to registers
		"                                \n\t"
		"                                \n\t"
		"mov    r0, %0                   \n\t" // k_loop
		"cmp    r0, #1                   \n\t"
		"                                \n\t"
		"ble    .DCLEANUP_T_8            \n\t"
		"                                \n\t"
		"                                \n\t"
		".DMAIN_LOOP2_T_8:                \n\t" // main loop
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)		
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q8, q4, q0           \n\t"
		"vmla.f32  q9, q5, q0           \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vmla.f32  q10, q6, q0           \n\t"
		"vmla.f32  q11, q7, q0           \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%1:128]!   \n\t" // load A0 to registers
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
		"add    %1, %1, %5               \n\t" // next band
		"vmla.f32  q12, q4, q0           \n\t"
		"vmla.f32  q13, q5, q0           \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vmla.f32  q14, q6, q0           \n\t"
		"vmla.f32  q15, q7, q0           \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%1:128]!   \n\t" // load A0 to registers
		"vmov   q0, q1                   \n\t"
		"vld1.64   {d2, d3}, [%2:128]!   \n\t" // load x to registers
		"                                \n\t"
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q8, q4, q0           \n\t"
		"vmla.f32  q9, q5, q0           \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vmla.f32  q10, q6, q0           \n\t"
		"vmla.f32  q11, q7, q0           \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%1:128]!   \n\t" // load A0 to registers
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
		"add    %1, %1, %5               \n\t" // next band
		"vmla.f32  q12, q4, q0           \n\t"
		"sub    r0, r0, #2               \n\t" // iter++
		"vmla.f32  q13, q5, q0           \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vmla.f32  q14, q6, q0           \n\t"
		"cmp    r0, #1                   \n\t" // next iter?
		"vmla.f32  q15, q7, q0           \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%1:128]!   \n\t" // load A0 to registers
		"vmov   q0, q1                   \n\t"
		"vld1.64   {d2, d3}, [%2:128]!   \n\t" // load x to registers
#endif		
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)	
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%1, r2]                 \n\t" // prefetch A1 to L1
#endif
		"vmla.f32  q8, q4, q0           \n\t"
		"vmla.f32  q9, q5, q0           \n\t"
		"vmla.f32  q10, q6, q0           \n\t"
		"vmla.f32  q11, q7, q0           \n\t"
		"sub    r0, r0, #2               \n\t" // iter++
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13, d14, d15}, [%1:128]!   \n\t" // load A0 to registers
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%1, r2]                 \n\t" // prefetch A1 to L1
#endif
		"vmla.f32  q12, q4, q0           \n\t"
		"vmla.f32  q13, q5, q0           \n\t"
		"vmla.f32  q14, q6, q0           \n\t"
		"vmla.f32  q15, q7, q0           \n\t"
		"vmov   q0, q1                   \n\t"
		"add    %1, %1, %5               \n\t" // next band
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13, d14, d15}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d2, d3}, [%2:128]!   \n\t" // load x to registers
		"                                \n\t"
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%1, r2]                 \n\t" // prefetch A1 to L1
#endif
		"vmla.f32  q8, q4, q0           \n\t"
		"vmla.f32  q9, q5, q0           \n\t"
		"vmla.f32  q10, q6, q0           \n\t"
		"vmla.f32  q11, q7, q0           \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13, d14, d15}, [%1:128]!   \n\t" // load A0 to registers
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%1, r2]                 \n\t" // prefetch A1 to L1
#endif
		"vmla.f32  q12, q4, q0           \n\t"
		"vmla.f32  q13, q5, q0           \n\t"
		"vmla.f32  q14, q6, q0           \n\t"
		"vmla.f32  q15, q7, q0           \n\t"
		"vmov   q0, q1                   \n\t"
		"add    %1, %1, %5               \n\t" // next band
		"cmp    r0, #1                   \n\t" // next iter?
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13, d14, d15}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d2, d3}, [%2:128]!   \n\t" // load x to registers
#endif		
		"                                \n\t"
		"bgt    .DMAIN_LOOP2_T_8          \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCLEANUP_T_8:                  \n\t"
		"                                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)		
		"vmov   q3, q2                   \n\t" // zero vector
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
		"vmov   s12, s0                  \n\t"
		"vmla.f32  d16, d8, d6            \n\t"
		"vmov   s13, s1                  \n\t"
		"vmla.f32  d18, d10, d6            \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vmov   s14, s2                  \n\t"
		"vmla.f32  q10, q6, q3           \n\t"
		"vmla.f32  q11, q7, q0           \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%1:128]!   \n\t" // load A0 to registers
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
		"add    %1, %1, %5               \n\t" // next band
		"vmla.f32  q12, q4, q0           \n\t"
		"add    %1, %1, #64              \n\t" // next band
		"vmla.f32  q13, q5, q0           \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vmla.f32  q14, q6, q0           \n\t"
		"vmla.f32  q15, q7, q0           \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%1:128]!   \n\t" // load A0 to registers
#endif		
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)	
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%1, r2]                 \n\t" // prefetch A1 to L1
#endif
		"vmov   q3, q2                   \n\t" // zero vector
		"vmov   s12, s0                  \n\t"
		"vmla.f32  d16, d8, d6            \n\t"
		"vmov   s13, s1                  \n\t"
		"vmla.f32  d18, d10, d6            \n\t"
		"vmov   s14, s2                  \n\t"
		"vmla.f32  q10, q6, q3           \n\t"
		"vmla.f32  q11, q7, q0           \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13, d14, d15}, [%1:128]!   \n\t" // load A0 to registers
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%1, r2]                 \n\t" // prefetch A1 to L1
#endif
		"add    %1, %1, %5               \n\t" // next band
		"vmla.f32  q12, q4, q0           \n\t"
		"vmla.f32  q13, q5, q0           \n\t"
		"vmla.f32  q14, q6, q0           \n\t"
		"vmla.f32  q15, q7, q0           \n\t"
		"add    %1, %1, #64              \n\t" // next band
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13, d14, d15}, [%1:128]!   \n\t" // load A0 to registers
#endif		
		"                                \n\t"
		"vmov   q3, q2                   \n\t" // zero vector
		"vmov   s12, s4                  \n\t"
		"vmla.f32  d24, d8, d6           \n\t"
		"vmov   s13, s5                  \n\t"
		"vmla.f32  d26, d10, d6           \n\t"
		"vmov   s14, s6                  \n\t"
		"vmla.f32  q14, q6, q3           \n\t"
		"vmla.f32  q15, q7, q1           \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vpadd.f32 d4, d16, d17          \n\t"
		"vpadd.f32 d5, d18, d19          \n\t"
		"vpadd.f32 d6, d20, d21          \n\t"
		"vpadd.f32 d7, d22, d23          \n\t"
		"vpadd.f32 d8, d24, d25          \n\t"
		"vpadd.f32 d9, d26, d27          \n\t"
		"vpadd.f32 d10, d28, d29         \n\t"
		"vpadd.f32 d11, d30, d31         \n\t"
		"                                \n\t"
		"vpadd.f32 d0, d4, d5            \n\t"
		"vpadd.f32 d1, d6, d7            \n\t"
		"vpadd.f32 d2, d8, d9            \n\t"
		"vpadd.f32 d3, d10, d11          \n\t"
		"                                \n\t"
		"                                \n\t"
		"cmp    %4, #0                   \n\t"
		"beq    .D0_T_8                  \n\t" // if alg==0, jump
		"                                \n\t"// alg==-1
		"mov    r0, %3                   \n\t" // load address of y
		"vld1.64   {d4, d5, d6, d7}, [r0]  \n\t" // load y
		"                                \n\t"
		"cmp    %4, #1                   \n\t"
		"beq    .D1_T_8                  \n\t" // if alg==1, jump
		"                                \n\t"
		"vsub.f32  q0, q2, q0            \n\t"
		"vsub.f32  q1, q3, q1            \n\t"
		"                                \n\t"
		"b      .D0_T_8                  \n\t" // jump to end
		"                                \n\t"
		"                                \n\t"
		".D1_T_8:                        \n\t" // alg==1
		"                                \n\t"
		"vadd.f32  q0, q2, q0            \n\t"
		"vadd.f32  q1, q3, q1            \n\t"
		"                                \n\t"
		".D0_T_8:                        \n\t" // alg==0
		"                                \n\t"
		"mov    r0, %3                   \n\t" // load address of y
		"                                \n\t"
		"vst1.64   {d0, d1, d2, d3}, [r0]  \n\t" // store y
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".align 3                        \n\t"
		".DZERO_T_8:                     \n\t" // zero quad word
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
		  "r" (A0),			// %1
		  "r" (x0),			// %2
		  "r" (y),			// %3
		  "r" (alg),		// %4
		  "r" (incA)		// %5
		: // register clobber list
		  "r0", "r1", "r2",
		  "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",
		  "d8", "d9", "d10", "d11", "d12", "d13", "d14", "d15",
		  "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23",
		  "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31",
		  "memory"
	);

	}



void kernel_strmv_u_t_4_lib4(int kmax, float *A, int sda, float *x, float *y, int alg)
	{

/*	if(kmax<=0) */
/*		return;*/
	
	const int bs = 4;

	float *A0 = A;
	float *x0 = x;
	
	__builtin_prefetch( A0 );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A0+8 );
#endif		

	int incA = bs*(sda-4)*sizeof(float);

	int k_iter = kmax/4;
	int k_left = kmax%4;

	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"add    r1, %5, #64              \n\t"
#if defined(TARGET_CORTEX_A9)
		"add    r2, %5, #32              \n\t"
#endif		
		"                                \n\t"
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%1, r2]                 \n\t" // prefetch A1 to L1
#endif
		"                                \n\t"
		"                                \n\t"
		"vldr   d16, .DZERO_T_4          \n\t" // load zero double
		"vldr   d17, .DZERO_T_4+8        \n\t" // load zero double
		"vmov   q9, q8                   \n\t"
		"vmov   q10, q8                  \n\t"
		"vmov   q11, q8                  \n\t"
		"vmov   q2, q8                   \n\t" // zero vector
		"                                \n\t"
		"                                \n\t"
		"vld1.64   {d0, d1, d2, d3}, [%2:128]!   \n\t" // load x to registers
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13, d14, d15}, [%1:128]!   \n\t" // load A0 to registers
		"                                \n\t"
		"                                \n\t"
		"mov    r0, %0                   \n\t" // k_loop
		"cmp    r0, #1                   \n\t"
		"                                \n\t"
		"beq    .DMAIN_LOOP_T_4          \n\t"
		"blt    .DCLEANUP_T_4            \n\t"
		"                                \n\t"
		"                                \n\t"
		".DMAIN_LOOP2_T_4:                \n\t" // main loop
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)		
		"add    %1, %1, %5               \n\t" // next band
		"vmla.f32  q8, q4, q0           \n\t"
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q9, q5, q0           \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vmla.f32  q10, q6, q0           \n\t"
		"vmla.f32  q11, q7, q0           \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%1:128]!   \n\t" // load A0 to registers
		"vmov   q0, q1                   \n\t"
		"vld1.64   {d2, d3}, [%2:128]!   \n\t" // load x to registers
		"                                \n\t"
		"add    %1, %1, %5               \n\t" // next band
		"vmla.f32  q8, q4, q0           \n\t"
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q9, q5, q0           \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vmla.f32  q10, q6, q0           \n\t"
		"sub    r0, r0, #2               \n\t" // iter++
		"vmla.f32  q11, q7, q0           \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%1:128]!   \n\t" // load A0 to registers
		"cmp    r0, #1                   \n\t" // next iter?
		"vmov   q0, q1                   \n\t"
		"vld1.64   {d2, d3}, [%2:128]!   \n\t" // load x to registers
#endif		
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)	
		"add    %1, %1, %5               \n\t" // next band
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%1, r2]                 \n\t" // prefetch A1 to L1
#endif
		"vmla.f32  q8, q4, q0           \n\t"
		"vmla.f32  q9, q5, q0           \n\t"
		"vmla.f32  q10, q6, q0           \n\t"
		"vmla.f32  q11, q7, q0           \n\t"
		"vmov   q0, q1                   \n\t"
		"sub    r0, r0, #2               \n\t" // iter++
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13, d14, d15}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d2, d3}, [%2:128]!   \n\t" // load x to registers
		"                                \n\t"
		"add    %1, %1, %5               \n\t" // next band
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%1, r2]                 \n\t" // prefetch A1 to L1
#endif
		"vmla.f32  q8, q4, q0           \n\t"
		"vmla.f32  q9, q5, q0           \n\t"
		"vmla.f32  q10, q6, q0           \n\t"
		"vmla.f32  q11, q7, q0           \n\t"
		"vmov   q0, q1                   \n\t"
		"cmp    r0, #1                   \n\t" // next iter?
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13, d14, d15}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d2, d3}, [%2:128]!   \n\t" // load x to registers
#endif		
		"                                \n\t"
		"bgt    .DMAIN_LOOP2_T_4          \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"blt    .DCLEANUP_T_4            \n\t"
		"                                \n\t"
		"                                \n\t"
		".DMAIN_LOOP_T_4:                \n\t" // main loop
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)		
		"add    %1, %1, %5               \n\t" // next band
		"vmla.f32  q8, q4, q0           \n\t"
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q9, q5, q0           \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vmla.f32  q10, q6, q0           \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"vmla.f32  q11, q7, q0           \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%1:128]!   \n\t" // load A0 to registers
		"cmp    r0, #0                   \n\t" // next iter?
		"vmov   q0, q1                   \n\t"
		"vld1.64   {d2, d3}, [%2:128]!   \n\t" // load x to registers
#endif		
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)	
		"add    %1, %1, %5               \n\t" // next band
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%1, r2]                 \n\t" // prefetch A1 to L1
#endif
		"vmla.f32  q8, q4, q0           \n\t"
		"vmla.f32  q9, q5, q0           \n\t"
		"vmla.f32  q10, q6, q0           \n\t"
		"vmla.f32  q11, q7, q0           \n\t"
		"vmov   q0, q1                   \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"cmp    r0, #0                   \n\t" // next iter?
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13, d14, d15}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d2, d3}, [%2:128]!   \n\t" // load x to registers
#endif		
		"                                \n\t"
		"bgt    .DMAIN_LOOP_T_4          \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCLEANUP_T_4:                  \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmov   q3, q2                   \n\t" // zero vector
		"                                \n\t"
		"vmov   s12, s0                  \n\t"
		"vmla.f32  d16, d8, d6           \n\t"
		"vmov   s13, s1                  \n\t"
		"vmla.f32  d18, d10, d6          \n\t"
		"vmov   s14, s2                  \n\t"
		"vmla.f32  q10, q6, q3           \n\t"
		"vmla.f32  q11, q7, q0           \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vpadd.f32 d4, d16, d17          \n\t"
		"vpadd.f32 d5, d18, d19          \n\t"
		"vpadd.f32 d6, d20, d21          \n\t"
		"vpadd.f32 d7, d22, d23          \n\t"
		"                                \n\t"
		"vpadd.f32 d0, d4, d5            \n\t"
		"vpadd.f32 d1, d6, d7            \n\t"
		"                                \n\t"
		"                                \n\t"
		"cmp    %4, #0                   \n\t"
		"beq    .D0_T_4                  \n\t" // if alg==0, jump
		"                                \n\t"// alg==-1
		"mov    r0, %3                   \n\t" // load address of y
		"vld1.64   {d4, d5}, [r0]        \n\t" // load y
		"                                \n\t"
		"cmp    %4, #1                   \n\t"
		"beq    .D1_T_4                  \n\t" // if alg==1, jump
		"                                \n\t"
		"vsub.f32  q0, q2, q0            \n\t"
		"                                \n\t"
		"b      .D0_T_4                  \n\t" // jump to end
		"                                \n\t"
		"                                \n\t"
		".D1_T_4:                        \n\t" // alg==1
		"                                \n\t"
		"vadd.f32  q0, q2, q0            \n\t"
		"                                \n\t"
		".D0_T_4:                        \n\t" // alg==0
		"                                \n\t"
		"mov    r0, %3                   \n\t" // load address of y
		"                                \n\t"
		"vst1.64   {d0, d1}, [r0]        \n\t" // store y
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".align 3                        \n\t"
		".DZERO_T_4:                     \n\t" // zero quad word
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
		  "r" (A0),			// %1
		  "r" (x0),			// %2
		  "r" (y),			// %3
		  "r" (alg),		// %4
		  "r" (incA)		// %5
		: // register clobber list
		  "r0", "r1", "r2",
		  "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",
		  "d8", "d9", "d10", "d11", "d12", "d13", "d14", "d15",
		  "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23",
		  "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31",
		  "memory"
	);

	}



void kernel_strmv_u_t_3_lib4(int kmax, float *A, int sda, float *x, float *y, int alg)
	{

/*	if(kmax<=0) */
/*		return;*/
	
	const int bs = 4;

	float *A0 = A;
	float *x0 = x;
	
	__builtin_prefetch( A0 );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A0+8 );
#endif		

	int incA = bs*(sda-3)*sizeof(float);

	int k_iter = kmax/4;
	int k_left = kmax%4;

	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"add    r1, %5, #48              \n\t"
#if defined(TARGET_CORTEX_A9)
		"add    r2, r1, #32              \n\t"
#endif		
		"                                \n\t"
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%1, r2]                 \n\t" // prefetch A1 to L1
#endif		
		"                                \n\t"
		"                                \n\t"
		"vldr   d16, .DZERO_T_3          \n\t" // load zero double
		"vldr   d17, .DZERO_T_3+8        \n\t" // load zero double
		"vmov   q9, q8                   \n\t"
		"vmov   q10, q8                  \n\t"
		"vmov   q2, q8                   \n\t" // zero vector
		"                                \n\t"
		"                                \n\t"
		"vld1.64   {d0, d1, d2, d3}, [%2:128]!   \n\t" // load x to registers
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13}, [%1:128]!   \n\t" // load A0 to registers
		"                                \n\t"
		"                                \n\t"
		"mov    r0, %0                   \n\t" // k_loop
		"cmp    r0, #1                   \n\t"
		"                                \n\t"
		"beq    .DMAIN_LOOP_T_3          \n\t"
		"blt    .DCLEANUP_T_3            \n\t"
		"                                \n\t"
		"                                \n\t"
		".DMAIN_LOOP2_T_3:                \n\t" // main loop
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)		
		"add    %1, %1, %5               \n\t" // next band
		"vmla.f32  q8, q4, q0           \n\t"
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q9, q5, q0           \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vmla.f32  q10, q6, q0           \n\t"
		"vld1.64   {d12, d13}, [%1:128]!   \n\t" // load A0 to registers
		"vmov   q0, q1                   \n\t"
		"vld1.64   {d2, d3}, [%2:128]!   \n\t" // load x to registers
		"                                \n\t"
		"add    %1, %1, %5               \n\t" // next band
		"vmla.f32  q8, q4, q0           \n\t"
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q9, q5, q0           \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vmla.f32  q10, q6, q0           \n\t"
		"sub    r0, r0, #2               \n\t" // iter++
		"vld1.64   {d12, d13}, [%1:128]!   \n\t" // load A0 to registers
		"cmp    r0, #1                   \n\t" // next iter?
		"vmov   q0, q1                   \n\t"
		"vld1.64   {d2, d3}, [%2:128]!   \n\t" // load x to registers
#endif		
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)	
		"add    %1, %1, %5               \n\t" // next band
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%1, r2]                 \n\t" // prefetch A1 to L1
#endif
		"vmla.f32  q8, q4, q0           \n\t"
		"vmla.f32  q9, q5, q0           \n\t"
		"vmla.f32  q10, q6, q0           \n\t"
		"vmov   q0, q1                   \n\t"
		"sub    r0, r0, #2               \n\t" // iter++
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d2, d3}, [%2:128]!   \n\t" // load x to registers
		"                                \n\t"
		"add    %1, %1, %5               \n\t" // next band
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%1, r2]                 \n\t" // prefetch A1 to L1
#endif
		"vmla.f32  q8, q4, q0           \n\t"
		"vmla.f32  q9, q5, q0           \n\t"
		"vmla.f32  q10, q6, q0           \n\t"
		"vmov   q0, q1                   \n\t"
		"cmp    r0, #1                   \n\t" // next iter?
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d2, d3}, [%2:128]!   \n\t" // load x to registers
#endif		
		"                                \n\t"
		"bgt    .DMAIN_LOOP2_T_3          \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"blt    .DCLEANUP_T_3            \n\t"
		"                                \n\t"
		"                                \n\t"
		".DMAIN_LOOP_T_3:                \n\t" // main loop
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)		
		"add    %1, %1, %5               \n\t" // next band
		"vmla.f32  q8, q4, q0           \n\t"
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q9, q5, q0           \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vmla.f32  q10, q6, q0           \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"vld1.64   {d12, d13}, [%1:128]!   \n\t" // load A0 to registers
		"cmp    r0, #0                   \n\t" // next iter?
		"vmov   q0, q1                   \n\t"
		"vld1.64   {d2, d3}, [%2:128]!   \n\t" // load x to registers
#endif		
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)	
		"add    %1, %1, %5               \n\t" // next band
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%1, r2]                 \n\t" // prefetch A1 to L1
#endif
		"sub    r0, r0, #1               \n\t" // iter++
		"vmla.f32  q8, q4, q0           \n\t"
		"vmla.f32  q9, q5, q0           \n\t"
		"vmla.f32  q10, q6, q0           \n\t"
		"vmov   q0, q1                   \n\t"
		"cmp    r0, #0                   \n\t" // next iter?
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d2, d3}, [%2:128]!   \n\t" // load x to registers
#endif		
		"                                \n\t"
		"bgt    .DMAIN_LOOP_T_3          \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCLEANUP_T_3:                  \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmov   q3, q2                   \n\t" // zero vector
		"                                \n\t"
		"vmov   s12, s0                  \n\t"
		"vmla.f32  d16, d8, d6           \n\t"
		"vmov   s13, s1                  \n\t"
		"vmla.f32  d18, d10, d6          \n\t"
		"vmov   s14, s2                  \n\t"
		"vmla.f32  q10, q6, q3           \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vpadd.f32 d4, d16, d17          \n\t"
		"vpadd.f32 d5, d18, d19          \n\t"
		"vpadd.f32 d6, d20, d21          \n\t"
		"                                \n\t"
		"vpadd.f32 d0, d4, d5            \n\t"
		"vadd.f32  s2, s12, s13          \n\t"
		"                                \n\t"
		"                                \n\t"
		"cmp    %4, #0                   \n\t"
		"beq    .D0_T_3                  \n\t" // if alg==0, jump
		"                                \n\t"// alg==-1
		"mov    r0, %3                   \n\t" // load address of y
		"vld1.64   {d4, d5}, [r0]        \n\t" // load y
		"                                \n\t"
		"cmp    %4, #1                   \n\t"
		"beq    .D1_T_3                  \n\t" // if alg==1, jump
		"                                \n\t"
		"vsub.f32  q0, q2, q0            \n\t"
		"                                \n\t"
		"b      .D0_T_3                  \n\t" // jump to end
		"                                \n\t"
		"                                \n\t"
		".D1_T_3:                        \n\t" // alg==1
		"                                \n\t"
		"vadd.f32  q0, q2, q0            \n\t"
		"                                \n\t"
		".D0_T_3:                        \n\t" // alg==0
		"                                \n\t"
		"mov    r0, %3                   \n\t" // load address of y
		"                                \n\t"
		"vmov      s3, s11               \n\t" // restore y[3]
		"vst1.64   {d0, d1}, [r0]        \n\t" // store y
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".align 3                        \n\t"
		".DZERO_T_3:                     \n\t" // zero quad word
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
		  "r" (A0),			// %1
		  "r" (x0),			// %2
		  "r" (y),			// %3
		  "r" (alg),		// %4
		  "r" (incA)		// %5
		: // register clobber list
		  "r0", "r1", "r2",
		  "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",
		  "d8", "d9", "d10", "d11", "d12", "d13", "d14", "d15",
		  "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23",
		  "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31",
		  "memory"
	);

	}



void kernel_strmv_u_t_2_lib4(int kmax, float *A, int sda, float *x, float *y, int alg)
	{
	
/*	if(kmax<=0) */
/*		return;*/
	
	const int bs = 4;

	float *A0 = A;
	float *x0 = x;
	
	__builtin_prefetch( A0 );

	int incA = bs*(sda-2)*sizeof(float);

	int k_iter = kmax/4;
	int k_left = kmax%4;

	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"add    r1, %5, #32              \n\t"
		"                                \n\t"
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
		"                                \n\t"
		"                                \n\t"
		"vldr   d16, .DZERO_T_2          \n\t" // load zero double
		"vldr   d17, .DZERO_T_2+8        \n\t" // load zero double
		"vmov   q9, q8                   \n\t"
		"vmov   q2, q8                   \n\t" // zero vector
		"                                \n\t"
		"                                \n\t"
		"vld1.64   {d0, d1, d2, d3}, [%2:128]!   \n\t" // load x to registers
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"                                \n\t"
		"                                \n\t"
		"mov    r0, %0                   \n\t" // k_loop
		"cmp    r0, #1                   \n\t"
		"                                \n\t"
		"beq    .DMAIN_LOOP_T_2          \n\t"
		"blt    .DCLEANUP_T_2            \n\t"
		"                                \n\t"
		"                                \n\t"
		".DMAIN_LOOP2_T_2:                \n\t" // main loop
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)		
		"add    %1, %1, %5               \n\t" // next band
		"vmla.f32  q8, q4, q0           \n\t"
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q9, q5, q0           \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vmov   q0, q1                   \n\t"
		"vld1.64   {d2, d3}, [%2:128]!   \n\t" // load x to registers
		"                                \n\t"
		"add    %1, %1, %5               \n\t" // next band
		"vmla.f32  q8, q4, q0           \n\t"
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q9, q5, q0           \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"sub    r0, r0, #2               \n\t" // iter++
		"cmp    r0, #1                   \n\t" // next iter?
		"vmov   q0, q1                   \n\t"
		"vld1.64   {d2, d3}, [%2:128]!   \n\t" // load x to registers
#endif		
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)	
		"add    %1, %1, %5               \n\t" // next band
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q8, q4, q0           \n\t"
		"vmla.f32  q9, q5, q0           \n\t"
		"vmov   q0, q1                   \n\t"
		"sub    r0, r0, #2               \n\t" // iter++
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d2, d3}, [%2:128]!   \n\t" // load x to registers
		"                                \n\t"
		"add    %1, %1, %5               \n\t" // next band
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q8, q4, q0           \n\t"
		"vmla.f32  q9, q5, q0           \n\t"
		"vmov   q0, q1                   \n\t"
		"cmp    r0, #1                   \n\t" // next iter?
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d2, d3}, [%2:128]!   \n\t" // load x to registers
#endif		
		"                                \n\t"
		"bgt    .DMAIN_LOOP2_T_2          \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"blt    .DCLEANUP_T_2            \n\t"
		"                                \n\t"
		"                                \n\t"
		".DMAIN_LOOP_T_2:                \n\t" // main loop
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)		
		"add    %1, %1, %5               \n\t" // next band
		"vmla.f32  q8, q4, q0           \n\t"
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q9, q5, q0           \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"sub    r0, r0, #1               \n\t" // iter++
		"cmp    r0, #0                   \n\t" // next iter?
		"vmov   q0, q1                   \n\t"
		"vld1.64   {d2, d3}, [%2:128]!   \n\t" // load x to registers
#endif		
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)	
		"add    %1, %1, %5               \n\t" // next band
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q8, q4, q0           \n\t"
		"vmla.f32  q9, q5, q0           \n\t"
		"vmov   q0, q1                   \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"cmp    r0, #0                   \n\t" // next iter?
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d2, d3}, [%2:128]!   \n\t" // load x to registers
#endif		
		"                                \n\t"
		"bgt    .DMAIN_LOOP_T_2          \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCLEANUP_T_2:                  \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmov   q3, q2                   \n\t" // zero vector
		"                                \n\t"
		"vmov   s12, s0                  \n\t"
		"vmla.f32  d16, d8, d6           \n\t"
		"vmla.f32  d18, d10, d0          \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vpadd.f32 d4, d16, d17          \n\t"
		"vpadd.f32 d5, d18, d19          \n\t"
		"                                \n\t"
		"vpadd.f32 d0, d4, d5            \n\t"
		"                                \n\t"
		"                                \n\t"
		"cmp    %4, #0                   \n\t"
		"beq    .D0_T_2                  \n\t" // if alg==0, jump
		"                                \n\t"// alg==-1
		"mov    r0, %3                   \n\t" // load address of y
		"vld1.64   {d4}, [r0]            \n\t" // load y
		"                                \n\t"
		"cmp    %4, #1                   \n\t"
		"beq    .D1_T_2                  \n\t" // if alg==1, jump
		"                                \n\t"
		"vsub.f32  d0, d4, d0            \n\t"
		"                                \n\t"
		"b      .D0_T_2                  \n\t" // jump to end
		"                                \n\t"
		"                                \n\t"
		".D1_T_2:                        \n\t" // alg==1
		"                                \n\t"
		"vadd.f32  d0, d4, d0            \n\t"
		"                                \n\t"
		".D0_T_2:                        \n\t" // alg==0
		"                                \n\t"
		"mov    r0, %3                   \n\t" // load address of y
		"                                \n\t"
		"vst1.64   {d0}, [r0]            \n\t" // store y
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".align 3                        \n\t"
		".DZERO_T_2:                     \n\t" // zero quad word
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
		  "r" (A0),			// %1
		  "r" (x0),			// %2
		  "r" (y),			// %3
		  "r" (alg),		// %4
		  "r" (incA)		// %5
		: // register clobber list
		  "r0", "r1", "r2",
		  "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",
		  "d8", "d9", "d10", "d11", "d12", "d13", "d14", "d15",
		  "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23",
		  "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31",
		  "memory"
	);

	}



void kernel_strmv_u_t_1_lib4(int kmax, float *A, int sda, float *x, float *y, int alg)
	{

/*	if(kmax<=0) */
/*		return;*/
	
	const int bs = 4;

	float *A0 = A;
	float *x0 = x;
	
	__builtin_prefetch( A0 );

	int incA = bs*(sda-1)*sizeof(float);

	int k_iter = kmax/4;
	int k_left = kmax%4;

	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"add    r1, %5, #16              \n\t"
		"                                \n\t"
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
		"                                \n\t"
		"                                \n\t"
		"vldr   d16, .DZERO_T_1          \n\t" // load zero double
		"vldr   d17, .DZERO_T_1+8        \n\t" // load zero double
		"vmov   q2, q8                   \n\t" // zero vector
		"                                \n\t"
		"                                \n\t"
		"vld1.64   {d0, d1, d2, d3}, [%2:128]!   \n\t" // load x to registers
		"vld1.64   {d8, d9}, [%1:128]!   \n\t" // load A0 to registers
		"                                \n\t"
		"                                \n\t"
		"mov    r0, %0                   \n\t" // k_loop
		"cmp    r0, #1                   \n\t"
		"                                \n\t"
		"beq    .DMAIN_LOOP_T_1          \n\t"
		"blt    .DCLEANUP_T_1            \n\t"
		"                                \n\t"
		"                                \n\t"
		".DMAIN_LOOP2_T_1:                \n\t" // main loop
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)		
		"add    %1, %1, %5               \n\t" // next band
		"vmla.f32  q8, q4, q0           \n\t"
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
		"vld1.64   {d8, d9}, [%1:128]!   \n\t" // load A0 to registers
		"vmov   q0, q1                   \n\t"
		"vld1.64   {d2, d3}, [%2:128]!   \n\t" // load x to registers
		"                                \n\t"
		"add    %1, %1, %5               \n\t" // next band
		"vmla.f32  q8, q4, q0           \n\t"
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
		"vld1.64   {d8, d9}, [%1:128]!   \n\t" // load A0 to registers
		"sub    r0, r0, #2               \n\t" // iter++
		"cmp    r0, #1                   \n\t" // next iter?
		"vmov   q0, q1                   \n\t"
		"vld1.64   {d2, d3}, [%2:128]!   \n\t" // load x to registers
#endif		
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)	
		"add    %1, %1, %5               \n\t" // next band
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q8, q4, q0           \n\t"
		"vmov   q0, q1                   \n\t"
		"sub    r0, r0, #2               \n\t" // iter++
		"vld1.64   {d8, d9}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d2, d3}, [%2:128]!   \n\t" // load x to registers
		"                                \n\t"
		"add    %1, %1, %5               \n\t" // next band
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q8, q4, q0           \n\t"
		"vmov   q0, q1                   \n\t"
		"cmp    r0, #1                   \n\t" // next iter?
		"vld1.64   {d8, d9}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d2, d3}, [%2:128]!   \n\t" // load x to registers
#endif		
		"                                \n\t"
		"bgt    .DMAIN_LOOP2_T_1          \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"blt    .DCLEANUP_T_1            \n\t"
		"                                \n\t"
		"                                \n\t"
		".DMAIN_LOOP_T_1:                \n\t" // main loop
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)		
		"add    %1, %1, %5               \n\t" // next band
		"vmla.f32  q8, q4, q0           \n\t"
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
		"vld1.64   {d8, d9}, [%1:128]!   \n\t" // load A0 to registers
		"sub    r0, r0, #1               \n\t" // iter++
		"cmp    r0, #0                   \n\t" // next iter?
		"vmov   q0, q1                   \n\t"
		"vld1.64   {d2, d3}, [%2:128]!   \n\t" // load x to registers
#endif		
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)	
		"add    %1, %1, %5               \n\t" // next band
		"pld    [%1, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q8, q4, q0           \n\t"
		"vmov   q0, q1                   \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"cmp    r0, #0                   \n\t" // next iter?
		"vld1.64   {d8, d9}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d2, d3}, [%2:128]!   \n\t" // load x to registers
#endif		
		"                                \n\t"
		"bgt    .DMAIN_LOOP_T_1          \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCLEANUP_T_1:                  \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmov   q3, q2                   \n\t" // zero vector
		"                                \n\t"
		"vmov   s12, s0                  \n\t"
		"vmla.f32  d16, d8, d6           \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vpadd.f32 d4, d16, d17          \n\t"
		"                                \n\t"
		"vadd.f32  s0, s8, s9            \n\t"
		"                                \n\t"
		"                                \n\t"
		"cmp    %4, #0                   \n\t"
		"beq    .D0_T_1                  \n\t" // if alg==0, jump
		"                                \n\t"// alg==-1
		"mov    r0, %3                   \n\t" // load address of y
		"vldr   s8, [r0, #0]             \n\t" // load y
		"                                \n\t"
		"cmp    %4, #1                   \n\t"
		"beq    .D1_T_1                  \n\t" // if alg==1, jump
		"                                \n\t"
		"vsub.f32  s0, s8, s0            \n\t"
		"                                \n\t"
		"b      .D0_T_1                  \n\t" // jump to end
		"                                \n\t"
		"                                \n\t"
		".D1_T_1:                        \n\t" // alg==1
		"                                \n\t"
		"vadd.f32  s0, s8, s0            \n\t"
		"                                \n\t"
		".D0_T_1:                        \n\t" // alg==0
		"                                \n\t"
		"mov    r0, %3                   \n\t" // load address of y
		"                                \n\t"
		"vstr   s0, [r0, #0]             \n\t" // store y
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".align 3                        \n\t"
		".DZERO_T_1:                     \n\t" // zero quad word
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
		  "r" (A0),			// %1
		  "r" (x0),			// %2
		  "r" (y),			// %3
		  "r" (alg),		// %4
		  "r" (incA)		// %5
		: // register clobber list
		  "r0", "r1", "r2",
		  "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",
		  "d8", "d9", "d10", "d11", "d12", "d13", "d14", "d15",
		  "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23",
		  "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31",
		  "memory"
	);

	}

	
	
void kernel_strmv_u_n_8_lib4(int kmax, float *A0, float *A1, float *x, float *y, int alg)
	{

	if(kmax<=0) 
		return;
	
	__builtin_prefetch( A0 );
	__builtin_prefetch( A1 );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A0+8 );
	__builtin_prefetch( A1+8 );
#endif		

	int k_iter = kmax/8;
	int k_left = kmax%8;

	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"pld    [%2, #64]                \n\t" // prefetch A0 to L1
		"pld    [%3, #64]                \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%2, #96]                \n\t" // prefetch A0 to L1
		"pld    [%3, #96]                \n\t" // prefetch A1 to L1
#endif		
		"                                \n\t"
		"                                \n\t"
		"vldr   d24, .DZERO_N_8          \n\t" // load zero double
		"vldr   d25, .DZERO_N_8+8        \n\t" // load zero double
		"vmov   q13, q12                 \n\t"
		"vmov   q14, q12                 \n\t"
		"vmov   q15, q12                 \n\t"
		"vmov   q4, q12                  \n\t"
		"vmov   q5, q12                  \n\t"
		"vmov   q6, q12                  \n\t"
		"vmov   q7, q12                  \n\t"
		"                                \n\t"
		"                                \n\t"
		"vld1.64   {d0, d1, d2, d3}, [%4:128]!   \n\t" // load x to registers
		"vldr      s16, [%2, #0]         \n\t"
		"vldr      d10, [%2, #16]        \n\t"
		"vldr      d12, [%2, #32]        \n\t"
		"vldr      s26, [%2, #40]        \n\t"
		"vldr      d14, [%2, #48]        \n\t"
		"vldr      d15, [%2, #56]        \n\t"
		"                                \n\t"
		"add       %2, #64               \n\t"
		"add       %3, #64               \n\t"
		"                                \n\t"
		"vld1.64   {d16, d17, d18, d19}, [%2:128]!   \n\t" // load A1 to registers
		"vld1.64   {d20, d21, d22, d23}, [%2:128]!   \n\t" // load A1 to registers
		"                                \n\t"
		"                                \n\t"
		"mov       r0, %0                \n\t" // k_iter
		"                                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)		
		"vld1.64   {d4, d5, d6, d7}, [%4:128]!   \n\t" // load x to registers
		"pld    [%2, #0]                 \n\t" // prefetch A0 to L1
		"vmla.f32  q12, q4, d0[0]        \n\t"
		"vldr      s16, [%3, #0]         \n\t"
		"vmla.f32  q13, q5, d0[1]        \n\t"
		"vldr      d10, [%3, #16]        \n\t"
		"pld    [%3, #64]                \n\t" // prefetch A1 to L1
		"vmla.f32  q12, q6, d1[0]        \n\t"
		"vldr      d12, [%3, #32]        \n\t"
		"vldr      s26, [%3, #40]        \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"vmla.f32  q13, q7, d1[1]        \n\t"
		"vldr      d14, [%3, #48]        \n\t"
		"vldr      d15, [%3, #56]        \n\t"
		"vmov   q0, q2                   \n\t"
		"add       %3, #64               \n\t"
		"                                \n\t"
		"pld    [%2, #64]                \n\t" // prefetch A1 to L1
		"vmla.f32  q14, q4, d2[0]        \n\t"
		"cmp    r0, #0                   \n\t" // next iter?
		"vmla.f32  q15, q5, d2[1]        \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%2:128]!   \n\t" // load A0 to registers
		"vmla.f32  q12, q8, d2[0]        \n\t"
		"pld    [%3, #64]                \n\t" // prefetch A1 to L1
		"vmla.f32  q13, q9, d2[1]        \n\t"
		"vld1.64   {d16, d17, d18, d19}, [%3:128]!   \n\t" // load A1 to registers
		"vmla.f32  q14, q6, d3[0]        \n\t"
		"vmla.f32  q15, q7, d3[1]        \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%2:128]!   \n\t" // load A0 to registers
		"vmla.f32  q12, q10, d3[0]       \n\t"
		"vmla.f32  q13, q11, d3[1]       \n\t"
		"vld1.64   {d20, d21, d22, d23}, [%3:128]!   \n\t" // load A1 to registers
		"vmov   q1, q3                   \n\t"
#endif		
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)
		"vld1.64   {d4, d5, d6, d7}, [%4:128]!   \n\t" // load x to registers
		"pld    [%2, #0]                 \n\t" // prefetch A0 to L1
		"pld    [%3, #64]                \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%2, #32]                 \n\t" // prefetch A0 to L1
		"pld    [%3, #96]                \n\t" // prefetch A1 to L1
#endif
		"vmla.f32  q12, q4, d0[0]        \n\t"
		"vldr      s16, [%3, #0]         \n\t"
		"vmla.f32  q13, q5, d0[1]        \n\t"
		"vldr      d10, [%3, #16]        \n\t"
		"vmla.f32  q12, q6, d1[0]        \n\t"
		"vldr      d12, [%3, #32]        \n\t"
		"vldr      s26, [%3, #40]        \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"vmla.f32  q13, q7, d1[1]        \n\t"
		"vldr      d14, [%3, #48]        \n\t"
		"vldr      d15, [%3, #56]        \n\t"
		"vmov   q0, q2                   \n\t"
		"add       %3, #64               \n\t"
		"                                \n\t"
		"pld    [%2, #64]                \n\t" // prefetch A1 to L1
		"pld    [%3, #64]                \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%2, #96]                \n\t" // prefetch A1 to L1
		"pld    [%3, #96]                \n\t" // prefetch A1 to L1
#endif
		"vmla.f32  q14, q4, d2[0]        \n\t"
		"vmla.f32  q15, q5, d2[1]        \n\t"
		"vmla.f32  q12, q8, d2[0]        \n\t"
		"vmla.f32  q13, q9, d2[1]        \n\t"
		"vmla.f32  q14, q6, d3[0]        \n\t"
		"vmla.f32  q15, q7, d3[1]        \n\t"
		"vmla.f32  q12, q10, d3[0]       \n\t"
		"vmla.f32  q13, q11, d3[1]       \n\t"
		"cmp    r0, #0                   \n\t" // next iter?
		"vld1.64   {d8, d9, d10, d11}, [%2:128]!   \n\t" // load A0 to registers
		"vld1.64   {d16, d17, d18, d19}, [%3:128]!   \n\t" // load A1 to registers
		"vld1.64   {d12, d13, d14, d15}, [%2:128]!   \n\t" // load A0 to registers
		"vld1.64   {d20, d21, d22, d23}, [%3:128]!   \n\t" // load A1 to registers
		"vmov   q1, q3                   \n\t"
#endif		
		"                                \n\t"
		"                                \n\t"
		"ble    .DCONSIDERLEFT4_N_8      \n\t"
		"                                \n\t"
		"                                \n\t"
		".DLOOPKITER_N_8:                \n\t" // main loop
		"                                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)		
		"vld1.64   {d4, d5, d6, d7}, [%4:128]!   \n\t" // load x to registers
		"pld    [%2, #64]                \n\t" // prefetch A0 to L1
		"vmla.f32  q12, q4, d0[0]         \n\t"
		"vmla.f32  q13, q5, d0[1]         \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%2:128]!   \n\t" // load A0 to registers
		"pld    [%3, #64]                \n\t" // prefetch A1 to L1
		"vmla.f32  q14, q8, d0[0]         \n\t"
		"vmla.f32  q15, q9, d0[1]         \n\t"
		"vld1.64   {d16, d17, d18, d19}, [%3:128]!   \n\t" // load A1 to registers
		"vmla.f32  q12, q6, d1[0]         \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"vmla.f32  q13, q7, d1[1]         \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%2:128]!   \n\t" // load A0 to registers
		"vmla.f32  q14, q10, d1[0]         \n\t"
		"vmla.f32  q15, q11, d1[1]         \n\t"
		"vld1.64   {d20, d21, d22, d23}, [%3:128]!   \n\t" // load A1 to registers
		"vmov   q0, q2                   \n\t"
		"                                \n\t"
		"pld    [%2, #64]                \n\t" // prefetch A1 to L1
		"vmla.f32  q12, q4, d2[0]         \n\t"
		"cmp    r0, #0                   \n\t" // next iter?
		"vmla.f32  q13, q5, d2[1]         \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%2:128]!   \n\t" // load A0 to registers
		"pld    [%3, #64]                \n\t" // prefetch A1 to L1
		"vmla.f32  q14, q8, d2[0]         \n\t"
		"vmla.f32  q15, q9, d2[1]         \n\t"
		"vld1.64   {d16, d17, d18, d19}, [%3:128]!   \n\t" // load A1 to registers
		"vmla.f32  q12, q6, d3[0]         \n\t"
		"vmla.f32  q13, q7, d3[1]         \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%2:128]!   \n\t" // load A0 to registers
		"vmla.f32  q14, q10, d3[0]         \n\t"
		"vmla.f32  q15, q11, d3[1]         \n\t"
		"vld1.64   {d20, d21, d22, d23}, [%3:128]!   \n\t" // load A1 to registers
		"vmov   q1, q3                   \n\t"
#endif		
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)	
		"vld1.64   {d4, d5, d6, d7}, [%4:128]!   \n\t" // load x to registers
		"pld    [%2, #64]                \n\t" // prefetch A0 to L1
		"pld    [%3, #64]                \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%2, #96]                \n\t" // prefetch A0 to L1
		"pld    [%3, #96]                \n\t" // prefetch A1 to L1
#endif
		"vmla.f32  q12, q4, d0[0]         \n\t"
		"vmla.f32  q13, q5, d0[1]         \n\t"
		"vmla.f32  q14, q8, d0[0]         \n\t"
		"vmla.f32  q15, q9, d0[1]         \n\t"
		"vmla.f32  q12, q6, d1[0]         \n\t"
		"vmla.f32  q13, q7, d1[1]         \n\t"
		"vmla.f32  q14, q10, d1[0]         \n\t"
		"vmla.f32  q15, q11, d1[1]         \n\t"
		"vmov   q0, q2                   \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"vld1.64   {d8, d9, d10, d11}, [%2:128]!   \n\t" // load A0 to registers
		"vld1.64   {d16, d17, d18, d19}, [%3:128]!   \n\t" // load A1 to registers
		"vld1.64   {d12, d13, d14, d15}, [%2:128]!   \n\t" // load A0 to registers
		"vld1.64   {d20, d21, d22, d23}, [%3:128]!   \n\t" // load A1 to registers
		"                                \n\t"
		"pld    [%2, #64]                \n\t" // prefetch A1 to L1
		"pld    [%3, #64]                \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%2, #96]                \n\t" // prefetch A1 to L1
		"pld    [%3, #96]                \n\t" // prefetch A1 to L1
#endif
		"vmla.f32  q12, q4, d2[0]         \n\t"
		"vmla.f32  q13, q5, d2[1]         \n\t"
		"vmla.f32  q14, q8, d2[0]         \n\t"
		"vmla.f32  q15, q9, d2[1]         \n\t"
		"vmla.f32  q12, q6, d3[0]         \n\t"
		"vmla.f32  q13, q7, d3[1]         \n\t"
		"vmla.f32  q14, q10, d3[0]         \n\t"
		"vmla.f32  q15, q11, d3[1]         \n\t"
		"vmov   q1, q3                   \n\t"
		"cmp    r0, #0                   \n\t" // next iter?
		"vld1.64   {d8, d9, d10, d11}, [%2:128]!   \n\t" // load A0 to registers
		"vld1.64   {d16, d17, d18, d19}, [%3:128]!   \n\t" // load A1 to registers
		"vld1.64   {d12, d13, d14, d15}, [%2:128]!   \n\t" // load A0 to registers
		"vld1.64   {d20, d21, d22, d23}, [%3:128]!   \n\t" // load A1 to registers
#endif		
		"                                \n\t"
		"                                \n\t"
		"bgt    .DLOOPKITER_N_8          \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERLEFT4_N_8:            \n\t" // consider left k+=2
		"                                \n\t"
		"mov    r0, %1                   \n\t" // k_left
		"cmp    r0, #3                   \n\t"
		"ble    .DCONSIDERLEFT2_N_8      \n\t"
		"                                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)		
		"vmla.f32  q12, q4, d0[0]         \n\t"
		"vmla.f32  q13, q5, d0[1]         \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%2:128]!   \n\t" // load A0 to registers
		"vmla.f32  q14, q8, d0[0]         \n\t"
		"vmla.f32  q15, q9, d0[1]         \n\t"
		"vld1.64   {d16, d17, d18, d19}, [%3:128]!   \n\t" // load A1 to registers
		"vmov   d0, d2                   \n\t"
		"vmla.f32  q12, q6, d1[0]         \n\t"
		"vmla.f32  q13, q7, d1[1]         \n\t"
		"vld1.64   {d12, d13}, [%2:128]!   \n\t" // load A0 to registers
		"sub    r0, r0, #4               \n\t" // k_left -= 4
		"vmla.f32  q14, q10, d1[0]         \n\t"
		"vmla.f32  q15, q11, d1[1]         \n\t"
		"vld1.64   {d20, d21}, [%3:128]!   \n\t" // load A1 to registers
		"vmov   d1, d3                   \n\t"
#endif		
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)	
		"vmla.f32  q12, q4, d0[0]         \n\t"
		"vmla.f32  q13, q5, d0[1]         \n\t"
		"vmla.f32  q14, q8, d0[0]         \n\t"
		"vmla.f32  q15, q9, d0[1]         \n\t"
		"vmla.f32  q12, q6, d1[0]         \n\t"
		"vmla.f32  q13, q7, d1[1]         \n\t"
		"vmla.f32  q14, q10, d1[0]         \n\t"
		"vmla.f32  q15, q11, d1[1]         \n\t"
		"vmov   q0, q1                   \n\t"
		"sub    r0, r0, #4               \n\t" // k_left -= 4
		"vld1.64   {d8, d9, d10, d11}, [%2:128]!   \n\t" // load A0 to registers
		"vld1.64   {d16, d17, d18, d19}, [%3:128]!   \n\t" // load A1 to registers
		"vld1.64   {d12, d13}, [%2:128]!   \n\t" // load A0 to registers
		"vld1.64   {d20, d21}, [%3:128]!   \n\t" // load A1 to registers
#endif		
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERLEFT2_N_8:            \n\t" // consider left k+=2
		"                                \n\t"
		"cmp    r0, #1                   \n\t"
		"ble    .DCONSIDERLEFT1_N_8      \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q12, q4, d0[0]        \n\t"
		"sub    r0, r0, #2               \n\t" // k_left -= 2
		"vmla.f32  q13, q5, d0[1]        \n\t"
		"vmov   q4, q6                   \n\t"
		"vmla.f32  q14, q8, d0[0]        \n\t"
		"vmla.f32  q15, q9, d0[1]       \n\t"
		"vmov   q8, q10                  \n\t"
		"vmov   d0, d1                   \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERLEFT1_N_8:            \n\t" // consider left k+=2
		"                                \n\t"
		"vadd.f32  q12, q12, q13         \n\t"
		"vadd.f32  q14, q14, q15         \n\t"
		"cmp    r0, #0                   \n\t"
		"ble    .DPOSTACCUM_N_8          \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q12, q4, d0[0]        \n\t"
		"vmla.f32  q14, q8, d0[0]        \n\t"
		"                                \n\t"
		"                                \n\t"
		".DPOSTACCUM_N_8:                \n\t"
		"                                \n\t"
		"cmp    %6, #0                   \n\t"
		"beq    .D0_N_8                  \n\t" // if alg==0, jump
		"                                \n\t"// alg==-1
		"mov    r0, %5                   \n\t" // load address of y
		"                                \n\t"
		"cmp    %6, #1                   \n\t"
		"beq    .D1_N_8                  \n\t" // if alg==1, jump
		"                                \n\t"
		"vld1.64   {d0, d1, d2, d3},   [r0:128]  \n\t" // load y
		"vsub.f32  q12, q0, q12          \n\t"
		"vsub.f32  q14, q1, q14          \n\t"
		"                                \n\t"
		"b      .D0_N_8                  \n\t" // jump to end
		"                                \n\t"
		"                                \n\t"
		".D1_N_8:                        \n\t" // alg==1
		"                                \n\t"
		"vld1.64   {d0, d1, d2, d3},   [r0:128]  \n\t" // load C0
		"vadd.f32  q12, q0, q12          \n\t"
		"vadd.f32  q14, q1, q14          \n\t"
		"                                \n\t"
		".D0_N_8:                        \n\t" // alg==0
		"                                \n\t"
		"mov    r0, %5                   \n\t" // load address of y
		"                                \n\t"
		"vst1.64   {d24, d25},     [r0:128]!  \n\t" // store y
		"vst1.64   {d28, d29},     [r0:128]   \n\t" // store y
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".align 3                        \n\t"
		".DZERO_N_8:                     \n\t" // zero quad word
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
		  "r" (x),			// %4
		  "r" (y),			// %5
		  "r" (alg)			// %6
		: // register clobber list
		  "r0", "r1", "r2",
		  "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",
		  "d8", "d9", "d10", "d11", "d12", "d13", "d14", "d15",
		  "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23",
		  "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31",
		  "memory"
	);
	
	}



void kernel_strmv_u_n_4_lib4(int kmax, float *A, float *x, float *y, int alg)
	{

	if(kmax<=0) 
		return;
	
	__builtin_prefetch( A );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A+8 );
#endif		
	
	kmax -= 4;
	int k_iter = kmax/8;
	int k_left = kmax%8;

	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"pld    [%2, #64]                \n\t" // prefetch A0 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%2, #96]                \n\t" // prefetch A0 to L1
#endif		
		"                                \n\t"
		"                                \n\t"
		"vldr   d24, .DZERO_N_4          \n\t" // load zero double
		"vldr   d25, .DZERO_N_4+8        \n\t" // load zero double
		"vmov   q13, q12                 \n\t"
		"vmov   q4, q12                  \n\t"
		"vmov   q5, q12                  \n\t"
		"vmov   q6, q12                  \n\t"
		"vmov   q7, q12                  \n\t"
		"                                \n\t"
		"                                \n\t"
		"vld1.64   {d0, d1}, [%3:128]!   \n\t" // load x to registers
		"vldr      s16, [%2, #0]         \n\t"
		"vldr      d10, [%2, #16]        \n\t"
		"vldr      d12, [%2, #32]        \n\t"
		"vldr      s26, [%2, #40]        \n\t"
		"vldr      d14, [%2, #48]        \n\t"
		"vldr      d15, [%2, #56]        \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"add       %3, #16               \n\t"*/
		"add       %2, #64               \n\t"
		"                                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)		
		"pld    [%2, #64]                \n\t" // prefetch A0 to L1
		"vmla.f32  q12, q4, d0[0]        \n\t"
		"vmla.f32  q13, q5, d0[1]        \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%2:128]!   \n\t" // load A0 to registers
		"vmla.f32  q12, q6, d1[0]        \n\t"
		"vmla.f32  q13, q7, d1[1]        \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%2:128]!   \n\t" // load A0 to registers
#endif		
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)	
		"pld    [%2, #64]                \n\t" // prefetch A0 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%2, #96]                \n\t" // prefetch A0 to L1
#endif
		"vmla.f32  q12, q4, d0[0]        \n\t"
		"vmla.f32  q13, q5, d0[1]        \n\t"
		"vmla.f32  q12, q6, d1[0]        \n\t"
		"vmla.f32  q13, q7, d1[1]        \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%2:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13, d14, d15}, [%2:128]!   \n\t" // load A0 to registers
#endif		
		"                                \n\t"
		"vld1.64   {d0, d1, d2, d3}, [%3:128]!   \n\t" // load x to registers
		"                                \n\t"
		"                                \n\t"
		"mov    r0, %0                   \n\t" // k_iter
		"cmp    r0, #0                   \n\t"
		"                                \n\t"
		"                                \n\t"
		"ble    .DCONSIDERLEFT4_N_4      \n\t"
		"                                \n\t"
		"                                \n\t"
		".DLOOPKITER_N_4:                \n\t" // main loop
		"                                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)		
		"vld1.64   {d4, d5, d6, d7}, [%3:128]!   \n\t" // load x to registers
		"pld    [%2, #64]                \n\t" // prefetch A0 to L1
		"vmla.f32  q12, q4, d0[0]         \n\t"
		"vmla.f32  q13, q5, d0[1]         \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%2:128]!   \n\t" // load A0 to registers
		"vmla.f32  q12, q6, d1[0]         \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"vmla.f32  q13, q7, d1[1]         \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%2:128]!   \n\t" // load A0 to registers
		"vmov   q0, q2                   \n\t"
		"                                \n\t"
		"pld    [%2, #64]                \n\t" // prefetch A0 to L1
		"vmla.f32  q12, q4, d2[0]         \n\t"
		"cmp    r0, #0                   \n\t" // next iter?
		"vmla.f32  q13, q5, d2[1]         \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%2:128]!   \n\t" // load A0 to registers
		"vmla.f32  q12, q6, d3[0]         \n\t"
		"vmla.f32  q13, q7, d3[1]         \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%2:128]!   \n\t" // load A0 to registers
		"vmov   q1, q3                   \n\t"
#endif		
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)	
		"vld1.64   {d4, d5, d6, d7}, [%3:128]!   \n\t" // load x to registers
		"pld    [%2, #64]                \n\t" // prefetch A0 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%2, #96]                \n\t" // prefetch A0 to L1
#endif
		"vmla.f32  q12, q4, d0[0]         \n\t"
		"vmla.f32  q13, q5, d0[1]         \n\t"
		"vmla.f32  q12, q6, d1[0]         \n\t"
		"vmla.f32  q13, q7, d1[1]         \n\t"
		"vmov   q0, q2                   \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"vld1.64   {d8, d9, d10, d11}, [%2:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13, d14, d15}, [%2:128]!   \n\t" // load A0 to registers
		"                                \n\t"
		"pld    [%2, #64]                \n\t" // prefetch A0 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%2, #96]                \n\t" // prefetch A0 to L1
#endif
		"vmla.f32  q12, q4, d2[0]         \n\t"
		"vmla.f32  q13, q5, d2[1]         \n\t"
		"vmla.f32  q12, q6, d3[0]         \n\t"
		"vmla.f32  q13, q7, d3[1]         \n\t"
		"vmov   q1, q3                   \n\t"
		"cmp    r0, #0                   \n\t" // next iter?
		"vld1.64   {d8, d9, d10, d11}, [%2:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13, d14, d15}, [%2:128]!   \n\t" // load A0 to registers
#endif		
		"                                \n\t"
		"                                \n\t"
		"bgt    .DLOOPKITER_N_4          \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERLEFT4_N_4:            \n\t" // consider left k+=2
		"                                \n\t"
		"mov    r0, %1                   \n\t" // k_left
		"cmp    r0, #3                   \n\t"
		"ble    .DCONSIDERLEFT2_N_4      \n\t"
		"                                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)		
		"vmla.f32  q12, q4, d0[0]         \n\t"
		"vmla.f32  q13, q5, d0[1]         \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%2:128]!   \n\t" // load A0 to registers
		"vmov   d0, d2                   \n\t"
		"vmla.f32  q12, q6, d1[0]         \n\t"
		"vmla.f32  q13, q7, d1[1]         \n\t"
		"vld1.64   {d12, d13}, [%2:128]!   \n\t" // load A0 to registers
		"sub    r0, r0, #4               \n\t" // k_left -= 4
		"vmov   d1, d3                   \n\t"
#endif		
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)	
		"vmla.f32  q12, q4, d0[0]         \n\t"
		"vmla.f32  q13, q5, d0[1]         \n\t"
		"vmla.f32  q12, q6, d1[0]         \n\t"
		"vmla.f32  q13, q7, d1[1]         \n\t"
		"vmov   q0, q1                   \n\t"
		"sub    r0, r0, #4               \n\t" // k_left -= 4
		"vld1.64   {d8, d9, d10, d11}, [%2:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13}, [%2:128]!   \n\t" // load A0 to registers
#endif		
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERLEFT2_N_4:            \n\t" // consider left k+=2
		"                                \n\t"
		"cmp    r0, #1                   \n\t"
		"ble    .DCONSIDERLEFT1_N_4      \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q12, q4, d0[0]        \n\t"
		"vmla.f32  q13, q5, d0[1]        \n\t"
		"sub    r0, r0, #2               \n\t" // k_left -= 2
		"vmov   q4, q6                   \n\t"
		"vmov   d0, d1                   \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERLEFT1_N_4:            \n\t" // consider left k+=2
		"                                \n\t"
		"vadd.f32  q12, q12, q13         \n\t"
		"cmp    r0, #0                   \n\t"
		"ble    .DPOSTACCUM_N_4          \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q12, q4, d0[0]        \n\t"
		"                                \n\t"
		"                                \n\t"
		".DPOSTACCUM_N_4:                \n\t"
		"                                \n\t"
		"cmp    %5, #0                   \n\t"
		"beq    .D0_N_4                  \n\t" // if alg==0, jump
		"                                \n\t"// alg==-1
		"mov    r0, %4                   \n\t" // load address of y
		"                                \n\t"
		"cmp    %5, #1                   \n\t"
		"beq    .D1_N_4                  \n\t" // if alg==1, jump
		"                                \n\t"
		"vld1.64   {d0, d1},   [r0:128]  \n\t" // load y
		"vsub.f32  q12, q0, q12          \n\t"
		"                                \n\t"
		"b      .D0_N_4                  \n\t" // jump to end
		"                                \n\t"
		"                                \n\t"
		".D1_N_4:                        \n\t" // alg==1
		"                                \n\t"
		"vld1.64   {d0, d1},   [r0:128]  \n\t" // load y
		"vadd.f32  q12, q0, q12          \n\t"
		"                                \n\t"
		".D0_N_4:                        \n\t" // alg==0
		"                                \n\t"
		"mov    r0, %4                   \n\t" // load address of y
		"                                \n\t"
		"vst1.64   {d24, d25},     [r0:128]!  \n\t" // store y
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".align 3                        \n\t"
		".DZERO_N_4:                     \n\t" // zero quad word
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
		  "r" (x),			// %3
		  "r" (y),			// %4
		  "r" (alg)			// %5
		: // register clobber list
		  "r0", "r1", "r2",
		  "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",
		  "d8", "d9", "d10", "d11", "d12", "d13", "d14", "d15",
		  "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23",
		  "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31",
		  "memory"
	);
	
	}



	
void kernel_strmv_u_n_2_lib4(int kmax, float *A, float *x, float *y, int alg)
	{

	if(kmax<=0) 
		return;
	
	const int lda = 4;
	
	int k;

	float
		x_0, x_1, x_2, x_3,
		y_0=0, y_1=0;
	
	x_0 = x[0];
	x_1 = x[1];

	y_0 += A[0+lda*0] * x_0;
/*	y_1 += A[1+lda*0] * x_0;*/

	y_0 += A[0+lda*1] * x_1;
	y_1 += A[1+lda*1] * x_1;
	
	A += 2*lda;
	x += 2;

	k=2;
	for(; k<kmax-3; k+=4)
		{

		x_0 = x[0];
		x_1 = x[1];
		x_2 = x[2];
		x_3 = x[3];

		y_0 += A[0+lda*0] * x_0;
		y_1 += A[1+lda*0] * x_0;

		y_0 += A[0+lda*1] * x_1;
		y_1 += A[1+lda*1] * x_1;

		y_0 += A[0+lda*2] * x_2;
		y_1 += A[1+lda*2] * x_2;

		y_0 += A[0+lda*3] * x_3;
		y_1 += A[1+lda*3] * x_3;
		
		A += 4*lda;
		x += 4;

		}

	for(; k<kmax; k++)
		{

		x_0 = x[0];

		y_0 += A[0+lda*0] * x_0;
		y_1 += A[1+lda*0] * x_0;
		
		A += 1*lda;
		x += 1;

		}

	if(alg==0)
		{
		y[0] = y_0;
		y[1] = y_1;
		}
	else if(alg==1)
		{
		y[0] += y_0;
		y[1] += y_1;
		}
	else // alg==-1
		{
		y[0] -= y_0;
		y[1] -= y_1;
		}

	}
	
	
	

