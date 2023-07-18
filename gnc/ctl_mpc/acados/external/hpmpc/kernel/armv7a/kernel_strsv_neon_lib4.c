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



void kernel_strsv_n_8_lib4(int kmax, float *A0, float *A1, float *x, float *y)
	{

/*	if(kmax<=0) */
/*		return;*/
	
	__builtin_prefetch( A0 );
	__builtin_prefetch( A1 );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A0+8 );
	__builtin_prefetch( A1+8 );
#endif		

	int k_iter = kmax/8;
/*	int k_left = kmax%8;*/

/*printf("\n%d\n", k_iter);*/
/*exit(1);*/

	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"pld    [%1, #64]                \n\t" // prefetch A0 to L1
		"pld    [%2, #64]                \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%1, #96]                \n\t" // prefetch A0 to L1
		"pld    [%2, #96]                \n\t" // prefetch A1 to L1
#endif		
		"                                \n\t"
		"                                \n\t"
		"mov    r0, %0                   \n\t" // k_iter
		"                                \n\t"
		"cmp    r0, #0                   \n\t"
		"                                \n\t"
		"                                \n\t"
		"vldr   d24, .DZERO_N_8          \n\t" // load zero double
		"vldr   d25, .DZERO_N_8+8        \n\t" // load zero double
		"vmov   q13, q12                 \n\t"
		"vmov   q14, q12                 \n\t"
		"vmov   q15, q12                 \n\t"
		"                                \n\t"
		"                                \n\t"
		"vld1.64   {d0, d1, d2, d3}, [%3:128]!   \n\t" // load x to registers
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d16, d17, d18, d19}, [%2:128]!   \n\t" // load A1 to registers
		"vld1.64   {d12, d13, d14, d15}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d20, d21, d22, d23}, [%2:128]!   \n\t" // load A1 to registers
		"                                \n\t"
		"                                \n\t"
		"ble    .DPOSTACCUM_N_8          \n\t"
		"                                \n\t"
		"                                \n\t"
		".DLOOPKITER_N_8:                \n\t" // main loop
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)		
		"                                \n\t"
		"vld1.64   {d4, d5, d6, d7}, [%3:128]!   \n\t" // load x to registers
		"pld    [%1, #64]                \n\t" // prefetch A0 to L1
		"vmla.f32  q12, q4, d0[0]         \n\t"
		"vmla.f32  q13, q5, d0[1]         \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"pld    [%2, #64]                \n\t" // prefetch A1 to L1
		"vmla.f32  q14, q8, d0[0]         \n\t"
		"vmla.f32  q15, q9, d0[1]         \n\t"
		"vld1.64   {d16, d17, d18, d19}, [%2:128]!   \n\t" // load A1 to registers
		"vmla.f32  q12, q6, d1[0]         \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"vmla.f32  q13, q7, d1[1]         \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%1:128]!   \n\t" // load A0 to registers
		"vmla.f32  q14, q10, d1[0]         \n\t"
		"vmla.f32  q15, q11, d1[1]         \n\t"
		"vld1.64   {d20, d21, d22, d23}, [%2:128]!   \n\t" // load A1 to registers
		"vmov   q0, q2                   \n\t"
		"                                \n\t"
		"pld    [%1, #64]                \n\t" // prefetch A1 to L1
		"vmla.f32  q12, q4, d2[0]         \n\t"
		"cmp    r0, #0                   \n\t" // next iter?
		"vmla.f32  q13, q5, d2[1]         \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"pld    [%2, #64]                \n\t" // prefetch A1 to L1
		"vmla.f32  q14, q8, d2[0]         \n\t"
		"vmla.f32  q15, q9, d2[1]         \n\t"
		"vld1.64   {d16, d17, d18, d19}, [%2:128]!   \n\t" // load A1 to registers
		"vmla.f32  q12, q6, d3[0]         \n\t"
		"vmla.f32  q13, q7, d3[1]         \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%1:128]!   \n\t" // load A0 to registers
		"vmla.f32  q14, q10, d3[0]         \n\t"
		"vmla.f32  q15, q11, d3[1]         \n\t"
		"vld1.64   {d20, d21, d22, d23}, [%2:128]!   \n\t" // load A1 to registers
		"vmov   q1, q3                   \n\t"
#endif		
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)	
		"                                \n\t"
		"vld1.64   {d4, d5, d6, d7}, [%3:128]!   \n\t" // load x to registers
		"pld    [%1, #64]                \n\t" // prefetch A0 to L1
		"pld    [%2, #64]                \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%1, #96]                \n\t" // prefetch A1 to L1
		"pld    [%2, #96]                \n\t" // prefetch A1 to L1
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
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d16, d17, d18, d19}, [%2:128]!   \n\t" // load A1 to registers
		"vld1.64   {d12, d13, d14, d15}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d20, d21, d22, d23}, [%2:128]!   \n\t" // load A1 to registers
		"                                \n\t"
		"pld    [%1, #64]                \n\t" // prefetch A1 to L1
		"pld    [%2, #64]                \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%1, #96]                \n\t" // prefetch A1 to L1
		"pld    [%2, #96]                \n\t" // prefetch A1 to L1
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
		"vld1.64   {d8, d9, d10, d11}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d16, d17, d18, d19}, [%2:128]!   \n\t" // load A1 to registers
		"vld1.64   {d12, d13, d14, d15}, [%1:128]!   \n\t" // load A0 to registers
		"vld1.64   {d20, d21, d22, d23}, [%2:128]!   \n\t" // load A1 to registers
		"                                \n\t"
#endif		
		"                                \n\t"
		"                                \n\t"
		"bgt    .DLOOPKITER_N_8          \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vadd.f32  q12, q12, q13         \n\t"
		"vadd.f32  q14, q14, q15         \n\t"
		"                                \n\t"
		"                                \n\t"
		".DPOSTACCUM_N_8:                \n\t"
		"                                \n\t"
		"vmov   q2, q4                   \n\t"
		"vmov   q3, q5                   \n\t"
		"vmov   q4, q8                   \n\t"
		"vmov   q5, q9                   \n\t"
		"vmov   q8, q6                   \n\t"
		"vmov   q9, q7                   \n\t"
		"                                \n\t"// alg==-1
		"mov    r0, %4                   \n\t" // load address of y
		"                                \n\t"
		"vld1.64   {d0, d1, d2, d3}, [r0]  \n\t" // load y
		"vsub.f32  q0, q0, q12           \n\t"
		"vsub.f32  q1, q1, q14           \n\t"
		"                                \n\t"
/*		"vst1.64   {d0, d1}, [%4]!  \n\t" // store y*/
/*		"vst1.64   {d2, d3}, [%4]!  \n\t" // store y*/
/*		"b .DEND                         \n\t"// alg==-1*/
		"                                \n\t"
		"                                \n\t"
/*		"vld1.64   {d4, d5, d6, d7}, [%1:128]!  \n\t" // load A0*/
/*		"vld1.64   {d8, d9, d10, d11}, [%2:128]!  \n\t" // load A1*/
		"                                \n\t"
		"vmul.f32  s24, s0, s8           \n\t"
		"vmls.f32  q0, q2, d12[0]        \n\t"
		"vmls.f32  q1, q4, d12[0]        \n\t"
		"                                \n\t"
		"vmul.f32  s25, s1, s13          \n\t"
/*		"vmls.f32  q0, q3, d12[1]        \n\t"*/
		"vmls.f32  d1, d7, d12[1]        \n\t"
		"vmls.f32  q1, q5, d12[1]        \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"vld1.64   {d4, d5, d6, d7}, [%1:128]!  \n\t" // load A0*/
/*		"vld1.64   {d8, d9, d10, d11}, [%2:128]!  \n\t" // load A1*/
		"vmov   q2, q8                   \n\t"
		"vmov   q3, q9                   \n\t"
		"vmov   q4, q10                  \n\t"
		"vmov   q5, q11                  \n\t"
		"                                \n\t"
		"vmul.f32  s26, s2, s10          \n\t"
/*		"vmls.f32  q0, q2, d13[0]        \n\t"*/
/*		"vmls.f32  d1, d5, d13[0]        \n\t"*/
		"vmls.f32  s3, s11, s26          \n\t"
		"vmls.f32  q1, q4, d13[0]        \n\t"
		"                                \n\t"
		"vmul.f32  s27, s3, s15          \n\t"
/*		"vmls.f32  q0, q3, d13[1]        \n\t"*/
/*		"vmls.f32  s3, s15, s27          \n\t"*/
		"vmls.f32  q1, q5, d13[1]        \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"vld1.64   {d4, d5, d6, d7}, [%1:128]!  \n\t" // load A0*/
		"vld1.64   {d8, d9, d10, d11}, [%2:128]!  \n\t" // load A1
		"                                \n\t"
		"vmul.f32  s28, s4, s16          \n\t"
/*		"vmls.f32  q0, q2, d12[0]        \n\t"*/
		"vmls.f32  q1, q4, d14[0]        \n\t"
		"                                \n\t"
		"vmul.f32  s29, s5, s21          \n\t"
/*		"vmls.f32  q0, q3, d12[1]        \n\t"*/
/*		"vmls.f32  q1, q5, d14[1]        \n\t"*/
		"vmls.f32  d3, d11, d14[1]       \n\t"
		"                                \n\t"
/*		"vld1.64   {d4, d5, d6, d7}, [%1:128]!  \n\t" // load A0*/
		"vld1.64   {d8, d9, d10, d11}, [%2:128]!  \n\t" // load A1
		"                                \n\t"
		"vmul.f32  s30, s6, s18          \n\t"
/*		"vmls.f32  q0, q2, d12[0]        \n\t"*/
/*		"vmls.f32  q1, q4, d15[0]        \n\t"*/
		"vmls.f32  s7, s19, s30           \n\t"
		"                                \n\t"
		"vmul.f32  s31, s7, s23          \n\t"
/*		"vmls.f32  q0, q3, d12[1]        \n\t"*/
/*		"vmls.f32  q1, q5, d15[1]        \n\t"*/
		"                                \n\t"
		"                                \n\t"
		"vst1.64   {d12, d13, d14, d15}, [r0]  \n\t" // store y
		"                                \n\t"
		"                                \n\t"
		".align 3                        \n\t"
		".DZERO_N_8:                     \n\t" // zero quad word
		".word  0                        \n\t"
		".word  0                        \n\t"
		".word  0                        \n\t"
		".word  0                        \n\t"
		"                                \n\t"
/*		".DEND:                          \n\t"*/
		"                                \n\t"
		: // output operands (none)
		: // input operands
		  "r" (k_iter),		// %0
		  "r" (A0),			// %1
		  "r" (A1),			// %2
		  "r" (x),			// %3
		  "r" (y)			// %4
		: // register clobber list
		  "r0", "r1", "r2",
		  "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",
		  "d8", "d9", "d10", "d11", "d12", "d13", "d14", "d15",
		  "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23",
		  "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31",
		  "memory"
	);
	
	}



void kernel_strsv_n_4_lib4(int kmax, int ksv, float *A, float *x, float *y)
	{

/*	if(kmax<=0) */
/*		return;*/
	
	__builtin_prefetch( A );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A+8 );
#endif		

	int k_iter = kmax/4;
/*	int k_left = kmax%8;*/

/*printf("\n%d %d\n", k_iter, ksv);*/
/*exit(1);*/

	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"pld    [%2, #64]                \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%2, #96]                \n\t" // prefetch A0 to L1
#endif		
		"                                \n\t"
		"                                \n\t"
		"mov    r0, %0                   \n\t" // k_iter
		"                                \n\t"
		"cmp    r0, #0                   \n\t"
		"                                \n\t"
		"                                \n\t"
		"vldr   d24, .DZERO_N_4          \n\t" // load zero double
		"vldr   d25, .DZERO_N_4+8        \n\t" // load zero double
		"vmov   q13, q12                 \n\t"
		"vmov   q14, q12                 \n\t"
		"vmov   q15, q12                 \n\t"
		"                                \n\t"
		"                                \n\t"
		"vld1.64   {d0, d1}, [%3:128]!   \n\t" // load x to registers
		"vld1.64   {d8, d9, d10, d11}, [%2:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13, d14, d15}, [%2:128]!   \n\t" // load A0 to registers
		"                                \n\t"
		"                                \n\t"
		"ble    .DPOSTACCUM_N_4          \n\t"
		"                                \n\t"
		"                                \n\t"
		".DLOOPKITER_N_4:                \n\t" // main loop
		"                                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)		
		"vmla.f32  q12, q4, d0[0]         \n\t"
		"vld1.64   {d2, d3}, [%3:128]!   \n\t" // load x to registers
		"vmla.f32  q13, q5, d0[1]         \n\t"
		"pld    [%2, #64]                \n\t" // prefetch A0 to L1
		"vld1.64   {d8, d9, d10, d11}, [%2:128]!   \n\t" // load A0 to registers
		"sub    r0, r0, #1               \n\t" // iter++
		"vmla.f32  q14, q6, d1[0]         \n\t"
		"cmp    r0, #0                   \n\t" // next iter?
		"vmla.f32  q15, q7, d1[1]         \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%2:128]!   \n\t" // load A0 to registers
		"vmov   q0, q1                   \n\t"
#endif		
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)	
		"vld1.64   {d2, d3}, [%3:128]!   \n\t" // load x to registers
		"pld    [%2, #64]                \n\t" // prefetch A0 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%2, #96]                \n\t" // prefetch A0 to L1
#endif
		"sub    r0, r0, #1               \n\t" // iter++
		"vmla.f32  q12, q4, d0[0]         \n\t"
		"vmla.f32  q13, q5, d0[1]         \n\t"
		"vmla.f32  q14, q6, d1[0]         \n\t"
		"vmla.f32  q15, q7, d1[1]         \n\t"
		"vmov   q0, q1                   \n\t"
		"cmp    r0, #0                   \n\t" // next iter?
		"vld1.64   {d8, d9, d10, d11}, [%2:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13, d14, d15}, [%2:128]!   \n\t" // load A0 to registers
#endif		
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"bgt    .DLOOPKITER_N_4          \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vadd.f32  q12, q12, q13         \n\t"
		"vadd.f32  q14, q14, q15         \n\t"
		"vadd.f32  q12, q12, q14         \n\t"
		"                                \n\t"
		"                                \n\t"
		".DPOSTACCUM_N_4:                \n\t"
		"                                \n\t"
		"vmov   q2, q4                   \n\t"
		"vmov   q3, q5                   \n\t"
/*		"vmov   q4, q8                   \n\t"*/
/*		"vmov   q5, q9                   \n\t"*/
		"vmov   q8, q6                   \n\t"
		"vmov   q9, q7                   \n\t"
		"                                \n\t"// alg==-1
		"mov    r0, %4                   \n\t" // load address of y
		"                                \n\t"
		"vld1.64   {d0, d1}, [r0]        \n\t" // load y
		"vsub.f32  q0, q0, q12           \n\t"
		"                                \n\t"
		"                                \n\t"
		"mov    r1, %1                   \n\t" // load ksv
		"                                \n\t"
		"cmp    r1, #1                   \n\t" // next iter?
		"                                \n\t"
/*		"vld1.64   {d4, d5, d6, d7}, [%2:128]!  \n\t" // load A0*/
		"                                \n\t"
		"vmul.f32  s24, s0, s8           \n\t"
		"vmls.f32  q0, q2, d12[0]        \n\t"
		"                                \n\t"
		"beq    .DKSV1_N_4               \n\t"
		"                                \n\t"
		"cmp    r1, #2                   \n\t" // next iter?
		"                                \n\t"
		"vmul.f32  s25, s1, s13          \n\t"
		"vmls.f32  d1, d7, d12[1]        \n\t"
		"                                \n\t"
		"beq    .DKSV2_N_4               \n\t"
		"                                \n\t"
		"cmp    r1, #3                   \n\t" // next iter?
		"                                \n\t"
/*		"vld1.64   {d4, d5, d6, d7}, [%2:128]!  \n\t" // load A0*/
		"vmov   q2, q8                   \n\t"
		"vmov   q3, q9                   \n\t"
/*		"vmov   q4, q10                  \n\t"*/
/*		"vmov   q5, q11                  \n\t"*/
		"                                \n\t"
		"vmul.f32  s26, s2, s10          \n\t"
		"vmls.f32  s3, s11, s26          \n\t"
		"                                \n\t"
		"beq    .DKSV3_N_4               \n\t"
		"                                \n\t"
		"vmul.f32  s27, s3, s15          \n\t"
		"                                \n\t"
		"                                \n\t"
		"b      .DSTORE_N_4              \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DKSV1_N_4:                     \n\t"
		"                                \n\t"
		"vmov   s25, s1                  \n\t"
		"                                \n\t"
		"                                \n\t"
		".DKSV2_N_4:                     \n\t"
		"                                \n\t"
		"vmov   s26, s2                  \n\t"
		"                                \n\t"
		"                                \n\t"
		".DKSV3_N_4:                     \n\t"
		"                                \n\t"
		"vmov   s27, s3                  \n\t"
		"                                \n\t"
		"                                \n\t"
		".DSTORE_N_4:                    \n\t"
		"                                \n\t"
		"vst1.64   {d12, d13}, [r0]      \n\t" // store y
		"                                \n\t"
		"                                \n\t"
		".align 3                        \n\t"
		".DZERO_N_4:                     \n\t" // zero quad word
		".word  0                        \n\t"
		".word  0                        \n\t"
		".word  0                        \n\t"
		".word  0                        \n\t"
		"                                \n\t"
/*		".DEND:                          \n\t"*/
		"                                \n\t"
		: // output operands (none)
		: // input operands
		  "r" (k_iter),		// %0
		  "r" (ksv),		// %1
		  "r" (A),			// %2
		  "r" (x),			// %3
		  "r" (y)			// %4
		: // register clobber list
		  "r0", "r1", "r2",
		  "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",
		  "d8", "d9", "d10", "d11", "d12", "d13", "d14", "d15",
		  "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23",
		  "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31",
		  "memory"
	);
	
	}
	
	
	
void kernel_strsv_t_4_lib4(int kmax, float *A, int sda, float *x)
	{

	if(kmax<=0) 
		return;
	
	const int bs = 4;

	__builtin_prefetch( A );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A+8 );
#endif	

	int incA = bs*(sda-4)*sizeof(float);

	kmax -= 4;
	int k_iter = kmax/4;
	int k_left = kmax%4;

	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"add    r1, %0, #64              \n\t"
#if defined(TARGET_CORTEX_A9)
		"add    r4, r1, #32              \n\t"
#endif	
		"                                \n\t"
		"pld    [%3, r1]                 \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%3, r4]                 \n\t" // prefetch A1 to L1
#endif	
		"                                \n\t"
		"                                \n\t"
		"mov    r2, %3                   \n\t" // backup A
		"mov    r3, %4                   \n\t" // backup x
		"                                \n\t"
		"                                \n\t"
		"add    %3, %3, r1               \n\t" // to next block
		"add    %4, %4, #16              \n\t"
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
		"mov    r0, %1                   \n\t" // k_loop
		"cmp    r0, #1                   \n\t"
		"                                \n\t"
		"                                \n\t"
		"mov    r5, %1                   \n\t" // k_loop plus clean
		"cmp    %2, #0                   \n\t"
		"addgt  r5, #1                   \n\t"
		"                                \n\t"
		"                                \n\t"
		"cmp    r5, #1                   \n\t"
		"blt    .DPOSTACCUM_T_4          \n\t"
		"                                \n\t"
		"vld1.64   {d0, d1}, [%4:128]!   \n\t" // load x to registers
		"vld1.64   {d8, d9, d10, d11}, [%3:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13, d14, d15}, [%3:128]!   \n\t" // load A0 to registers
		"add    %3, %3, %0               \n\t" // next band
		"                                \n\t"
		"                                \n\t"
		"cmp    r5, #2                   \n\t"
		"ble    .DCONS_MAIN_LOOP_T_4     \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DMAIN_LOOP2_T_4:                \n\t" // main loop
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)		
		"vld1.64   {d2, d3}, [%4:128]!   \n\t" // load x to registers
		"vmla.f32  q8, q4, q0           \n\t"
		"pld    [%3, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q9, q5, q0           \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%3:128]!   \n\t" // load A0 to registers
		"vmla.f32  q10, q6, q0           \n\t"
		"sub    r5, r5, #2               \n\t" // iter++
		"vmla.f32  q11, q7, q0           \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%3:128]!   \n\t" // load A0 to registers
		"add    %3, %3, %0               \n\t" // next band
		"                                \n\t"
		"vld1.64   {d0, d1}, [%4:128]!   \n\t" // load x to registers
		"vmla.f32  q8, q4, q1           \n\t"
		"pld    [%3, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q9, q5, q1           \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%3:128]!   \n\t" // load A0 to registers
		"vmla.f32  q10, q6, q1           \n\t"
		"cmp    r5, #2                   \n\t" // next iter?
		"vmla.f32  q11, q7, q1           \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%3:128]!   \n\t" // load A0 to registers
		"add    %3, %3, %0               \n\t" // next band
#endif		
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)	
		"pld    [%3, r1]                 \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%3, r4]                 \n\t" // prefetch A1 to L1
#endif
		"vmla.f32  q8, q4, q0           \n\t"
		"vmla.f32  q9, q5, q0           \n\t"
		"vmla.f32  q10, q6, q0           \n\t"
		"vmla.f32  q11, q7, q0           \n\t"
		"sub    r5, r5, #2               \n\t" // iter++
		"vld1.64   {d0, d1}, [%4:128]!   \n\t" // load x to registers
		"vld1.64   {d8, d9, d10, d11}, [%3:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13, d14, d15}, [%3:128]!   \n\t" // load A0 to registers
		"add    %3, %3, %0               \n\t" // next band
		"                                \n\t"
		"pld    [%3, r1]                 \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%3, r4]                 \n\t" // prefetch A1 to L1
#endif
		"vmla.f32  q8, q4, q0           \n\t"
		"vmla.f32  q9, q5, q0           \n\t"
		"vmla.f32  q10, q6, q0           \n\t"
		"vmla.f32  q11, q7, q0           \n\t"
		"cmp    r5, #2                   \n\t" // next iter?
		"vld1.64   {d0, d1}, [%4:128]!   \n\t" // load x to registers
		"vld1.64   {d8, d9, d10, d11}, [%3:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13, d14, d15}, [%3:128]!   \n\t" // load A0 to registers
		"add    %3, %3, %0               \n\t" // next band
#endif		
		"                                \n\t"
		"bgt    .DMAIN_LOOP2_T_4          \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONS_MAIN_LOOP_T_4:           \n\t" // main loop
		"                                \n\t"
		"blt    .DCONS_CLEAN_LOOP_T_4    \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DMAIN_LOOP_T_4:                \n\t" // main loop
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)		
		"vld1.64   {d2, d3}, [%4:128]!   \n\t" // load x to registers
		"vmla.f32  q8, q4, q0           \n\t"
		"pld    [%3, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q9, q5, q0           \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%3:128]!   \n\t" // load A0 to registers
		"vmla.f32  q10, q6, q0           \n\t"
		"vmla.f32  q11, q7, q0           \n\t"
		"vld1.64   {d12, d13, d14, d15}, [%3:128]!   \n\t" // load A0 to registers
//		"add    %3, %3, %0               \n\t" // next band
		"vmov   q0, q1                   \n\t"
#endif		
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)	
		"pld    [%3, r1]                 \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%3, r4]                 \n\t" // prefetch A1 to L1
#endif
		"vmla.f32  q8, q4, q0           \n\t"
		"vmla.f32  q9, q5, q0           \n\t"
		"vmla.f32  q10, q6, q0           \n\t"
		"vmla.f32  q11, q7, q0           \n\t"
		"vmov   q0, q1                   \n\t"
		"vld1.64   {d0, d1}, [%4:128]!   \n\t" // load x to registers
		"vld1.64   {d8, d9, d10, d11}, [%3:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13, d14, d15}, [%3:128]!   \n\t" // load A0 to registers
//		"add    %3, %3, %0               \n\t" // next band
#endif		
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONS_CLEAN_LOOP_T_4:          \n\t" // main loop
		"                                \n\t"
		"cmp    %2, #0                   \n\t"
		"ble    .DCLEAN_LOOP_T_4         \n\t"
		"                                \n\t"
		"vmov   s3, s8                   \n\t"
		"cmp    r0, #2                   \n\t"
		"vmoveq s2, s8                   \n\t"
		"vmovlt s1, s8                   \n\t"
		"                                \n\t"
		".DCLEAN_LOOP_T_4:               \n\t" // main loop
		"                                \n\t"
		"vmla.f32  q8, q4, q0            \n\t"
		"vmla.f32  q9, q5, q0            \n\t"
		"vmla.f32  q10, q6, q0           \n\t"
		"vmla.f32  q11, q7, q0           \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DPOSTACCUM_T_4:                \n\t"
		"                                \n\t"
		"pld    [r2, #0]                 \n\t" // prefetch A to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [r2, #32]                \n\t" // prefetch A to L1
#endif		
		"                                \n\t"
		"mov    r0, r3                   \n\t" // load address of x[0]
		"vld1.64   {d2, d3}, [r0:128]    \n\t" // load x[0]
		"vld1.64   {d8, d9, d10, d11}, [r2:128]!   \n\t" // load A to registers
		"vld1.64   {d12, d13, d14, d15}, [r2:128]! \n\t" // load A to registers
		"                                \n\t"
		"                                \n\t"
		// bottom trinagle
		"vpadd.f32 d6, d20, d21          \n\t"
		"vpadd.f32 d7, d22, d23          \n\t"
		"                                \n\t"
		"vpadd.f32 d1, d6, d7            \n\t"
		"                                \n\t"
		"vsub.f32  d1, d3, d1            \n\t"
		"                                \n\t"
		"vmul.f32  s3, s31, s3           \n\t"
		"                                \n\t"
		"vmls.f32  s2, s27, s3           \n\t"
		"vmul.f32  s2, s26, s2           \n\t"
		"                                \n\t"
		"                                \n\t"
		// square
		"vmla.f32  d16, d9, d1           \n\t"
		"vmla.f32  d18, d11, d1          \n\t"
		"                                \n\t"
		"                                \n\t"
		// top trinagle
		"vpadd.f32 d4, d16, d17          \n\t"
		"vpadd.f32 d5, d18, d19          \n\t"
		"                                \n\t"
		"vpadd.f32 d0, d4, d5            \n\t"
		"                                \n\t"
		"vsub.f32  d0, d2, d0            \n\t"
		"                                \n\t"
		"vmul.f32  s1, s21, s1           \n\t"
		"                                \n\t"
		"vmls.f32  s0, s17, s1           \n\t"
		"vmul.f32  s0, s16, s0           \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"mov    r0, r3                   \n\t" // load address of x[0] */
		"vst1.64   {d0, d1}, [r0:128]    \n\t" // store x[0]
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
		  "r" (incA),		// %0
		  "r" (k_iter),		// %1
		  "r" (k_left),		// %2
		  "r" (A),			// %3
		  "r" (x)			// %4
		: // register clobber list
		  "r0", "r1", "r2", "r3", "r4", "r5",
		  "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",
		  "d8", "d9", "d10", "d11", "d12", "d13", "d14", "d15",
		  "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23",
		  "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31",
		  "memory"
	);

	}



void kernel_strsv_t_3_lib4(int kmax, float *A, int sda, float *x)
	{

	if(kmax<=0) 
		return;
	
	const int bs = 4;

	__builtin_prefetch( A );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A+8 );
#endif		

	int incA = bs*(sda-3)*sizeof(float);

	kmax -= 4;
	int k_iter = kmax/4;
	int k_left = kmax%4;

	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"add    r1, %0, #48              \n\t"
#if defined(TARGET_CORTEX_A9)
		"add    r4, r1, #32              \n\t"
#endif		
		"                                \n\t"
		"pld    [%3, r1]                 \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%3, r4]                 \n\t" // prefetch A1 to L1
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
		"mov    r2, %3                   \n\t" // backup A
		"mov    r3, %4                   \n\t" // backup x
		"                                \n\t"
		"                                \n\t"
		// cleanup at the beginning
		"vld1.64   {d0, d1}, [%4:128]    \n\t" // load x to registers
		"vld1.64   {d8, d9, d10, d11}, [%3:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13}, [%3:128]! \n\t" // load A0 to registers
		"vmov      s2, s8                \n\t" // zeros x[2]
		"vmla.f32  d16, d9, d1           \n\t"
		"vmla.f32  d18, d11, d1          \n\t"
		"vmla.f32  d20, d13, d1          \n\t"
		"                                \n\t"
		"mov    %3, r2                   \n\t" // restore A
		"                                \n\t"
		"                                \n\t"
		"add    %3, %3, r1               \n\t" // to next block
		"add    %4, %4, #16              \n\t"
		"                                \n\t"
		"                                \n\t"
		"mov    r0, %1                   \n\t" // k_loop
		"cmp    r0, #1                   \n\t"
		"                                \n\t"
		"                                \n\t"
		"vld1.64   {d0, d1, d2, d3}, [%4:128]!   \n\t" // load x to registers
		"vld1.64   {d8, d9, d10, d11}, [%3:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13}, [%3:128]!   \n\t" // load A0 to registers
		"                                \n\t"
		"                                \n\t"
		"beq    .DMAIN_LOOP_T_3          \n\t"
		"blt    .DCONS_CLEAN_LOOP_T_3    \n\t"
		"                                \n\t"
		"                                \n\t"
		".DMAIN_LOOP2_T_3:                \n\t" // main loop
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)		
		"add    %3, %3, %0               \n\t" // next band
		"vmla.f32  q8, q4, q0           \n\t"
		"pld    [%3, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q9, q5, q0           \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%3:128]!   \n\t" // load A0 to registers
		"vmla.f32  q10, q6, q0           \n\t"
		"vld1.64   {d12, d13}, [%3:128]!   \n\t" // load A0 to registers
		"vld1.64   {d0, d1}, [%4:128]!   \n\t" // load x to registers
		"                                \n\t"
		"add    %3, %3, %0               \n\t" // next band
		"vmla.f32  q8, q4, q1           \n\t"
		"pld    [%3, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q9, q5, q1           \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%3:128]!   \n\t" // load A0 to registers
		"vmla.f32  q10, q6, q1           \n\t"
		"sub    r0, r0, #2               \n\t" // iter++
		"vld1.64   {d12, d13}, [%3:128]!   \n\t" // load A0 to registers
		"cmp    r0, #1                   \n\t" // next iter?
		"vld1.64   {d2, d3}, [%4:128]!   \n\t" // load x to registers
#endif		
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)	
		"pld    [%3, r1]                 \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%3, r4]                 \n\t" // prefetch A1 to L1
#endif
		"vmla.f32  q8, q4, q0           \n\t"
		"vmla.f32  q9, q5, q0           \n\t"
		"vmla.f32  q10, q6, q0           \n\t"
		"add    %3, %3, %0               \n\t" // next band
		"sub    r0, r0, #2               \n\t" // iter++
		"vld1.64   {d8, d9, d10, d11}, [%3:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13}, [%3:128]!   \n\t" // load A0 to registers
		"vld1.64   {d0, d1}, [%4:128]!   \n\t" // load x to registers
		"                                \n\t"
		"pld    [%3, r1]                 \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%3, r4]                 \n\t" // prefetch A1 to L1
#endif
		"vmla.f32  q8, q4, q1           \n\t"
		"vmla.f32  q9, q5, q1           \n\t"
		"vmla.f32  q10, q6, q1           \n\t"
		"add    %3, %3, %0               \n\t" // next band
		"cmp    r0, #1                   \n\t" // next iter?
		"vld1.64   {d8, d9, d10, d11}, [%3:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13}, [%3:128]!   \n\t" // load A0 to registers
		"vld1.64   {d2, d3}, [%4:128]!   \n\t" // load x to registers
#endif		
		"                                \n\t"
		"bgt    .DMAIN_LOOP2_T_3          \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"blt    .DCONS_CLEAN_LOOP_T_3    \n\t"
		"                                \n\t"
		"                                \n\t"
		".DMAIN_LOOP_T_3:                \n\t" // main loop
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)		
		"add    %3, %3, %0               \n\t" // next band
		"vmla.f32  q8, q4, q0           \n\t"
		"pld    [%3, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q9, q5, q0           \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%3:128]!   \n\t" // load A0 to registers
		"vmla.f32  q10, q6, q0           \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"vld1.64   {d12, d13}, [%3:128]!   \n\t" // load A0 to registers
		"cmp    r0, #0                   \n\t" // next iter?
		"vmov   q0, q1                   \n\t"
		"vld1.64   {d2, d3}, [%4:128]!   \n\t" // load x to registers
#endif		
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)	
		"pld    [%3, r1]                 \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%3, r4]                 \n\t" // prefetch A1 to L1
#endif
		"vmla.f32  q8, q4, q0           \n\t"
		"vmla.f32  q9, q5, q0           \n\t"
		"vmla.f32  q10, q6, q0           \n\t"
		"vmov   q0, q1                   \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"add    %3, %3, %0               \n\t" // next band
		"cmp    r0, #0                   \n\t" // next iter?
		"vld1.64   {d8, d9, d10, d11}, [%3:128]!   \n\t" // load A0 to registers
		"vld1.64   {d12, d13}, [%3:128]!   \n\t" // load A0 to registers
		"vld1.64   {d2, d3}, [%4:128]!   \n\t" // load x to registers
#endif		
		"                                \n\t"
		"bgt    .DMAIN_LOOP_T_3          \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONS_CLEAN_LOOP_T_3:          \n\t" // main loop
		"                                \n\t"
		"mov    r0, %2                   \n\t" // k_left
		"cmp    r0, #0                   \n\t"
		"                                \n\t"
		"ble    .DPOSTACCUM_T_3          \n\t"
		"                                \n\t"
		"vmov   s3, s8                   \n\t"
		"cmp    r0, #2                   \n\t"
		"vmoveq s2, s8                   \n\t"
		"vmovlt s1, s8                   \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q8, q4, q0            \n\t"
		"vmla.f32  q9, q5, q0            \n\t"
		"vmla.f32  q10, q6, q0           \n\t"
		"                                \n\t"
		"                                \n\t"
		".DPOSTACCUM_T_3:                \n\t"
		"                                \n\t"
		"                                \n\t"
		"pld    [r2, #0]                 \n\t" // prefetch A to L1
		"                                \n\t"
		"                                \n\t"
		"mov    r0, r3                   \n\t" // load address of x[0]
		"vld1.64   {d2, d3}, [r0:128]    \n\t" // load x[0]
		"                                \n\t"
		"                                \n\t"
		"vld1.64   {d8, d9, d10, d11}, [r2:128]!   \n\t" // load A to registers
		"vld1.64   {d12, d13}, [r2:128]! \n\t" // load A to registers
		"                                \n\t"
		"                                \n\t"
		// bottom trinagle
		"vpadd.f32 d6, d20, d21          \n\t"
		"                                \n\t"
		"vadd.f32  s2, s12, s13          \n\t"
		"                                \n\t"
		"vsub.f32  s2, s6, s2            \n\t"
		"                                \n\t"
		"vmul.f32  s2, s26, s2           \n\t"
		"                                \n\t"
		"                                \n\t"
		// top trinagle & square
		"vpadd.f32 d4, d16, d17          \n\t"
		"vpadd.f32 d5, d18, d19          \n\t"
		"                                \n\t"
		"vpadd.f32 d0, d4, d5            \n\t"
		"                                \n\t"
		"vmla.f32  s0, s18, s2           \n\t"
		"vmla.f32  s1, s22, s2          \n\t"
		"                                \n\t"
		"vsub.f32  d0, d2, d0            \n\t"
		"                                \n\t"
		"vmul.f32  s1, s21, s1           \n\t"
		"                                \n\t"
		"vmls.f32  s0, s17, s1           \n\t"
		"vmul.f32  s0, s16, s0           \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"mov    r0, r3                   \n\t" // load address of x[0] */
		"vmov      s3, s7                \n\t" // restore x[3]
		"vst1.64   {d0, d1}, [r0:128]    \n\t" // store x[0]
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
		  "r" (incA),		// %0
		  "r" (k_iter),		// %1
		  "r" (k_left),		// %2
		  "r" (A),			// %3
		  "r" (x)			// %4
		: // register clobber list
		  "r0", "r1", "r2", "r3", "r4",
		  "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",
		  "d8", "d9", "d10", "d11", "d12", "d13", "d14", "d15",
		  "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23",
		  "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31",
		  "memory"
	);

	}



void kernel_strsv_t_2_lib4(int kmax, float *A, int sda, float *x)
	{

	if(kmax<=0) 
		return;
	
	const int bs = 4;

	__builtin_prefetch( A );

	int incA = bs*(sda-2)*sizeof(float);

	kmax -= 4;
	int k_iter = kmax/4;
	int k_left = kmax%4;

	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"add    r1, %0, #32              \n\t"
		"                                \n\t"
		"pld    [%3, r1]                 \n\t" // prefetch A1 to L1
		"                                \n\t"
		"                                \n\t"
		"vldr   d16, .DZERO_T_2          \n\t" // load zero double
		"vldr   d17, .DZERO_T_2+8        \n\t" // load zero double
		"vmov   q9, q8                   \n\t"
		"vmov   q2, q8                   \n\t" // zero vector
		"                                \n\t"
		"                                \n\t"
		"mov    r2, %3                   \n\t" // backup A
		"mov    r3, %4                   \n\t" // backup x
		"                                \n\t"
		"                                \n\t"
		// clean up at the beginning
		"vld1.64   {d0, d1}, [%4:128]   \n\t" // load x to registers
		"vld1.64   {d8, d9, d10, d11}, [%3:128]   \n\t" // load A0 to registers
		"                                \n\t"
		"vmla.f32  d16, d9, d1           \n\t"
		"vmla.f32  d18, d11, d1           \n\t"
		"                                \n\t"
		"                                \n\t"
		"add    %3, %3, r1               \n\t" // to next block
		"add    %4, %4, #16              \n\t"
		"                                \n\t"
		"                                \n\t"
		"mov    r0, %1                   \n\t" // k_loop
		"cmp    r0, #1                   \n\t"
		"                                \n\t"
		"                                \n\t"
		"vld1.64   {d0, d1, d2, d3}, [%4:128]!   \n\t" // load x to registers
		"vld1.64   {d8, d9, d10, d11}, [%3:128]!   \n\t" // load A0 to registers
		"                                \n\t"
		"                                \n\t"
		"beq    .DMAIN_LOOP_T_2          \n\t"
		"blt    .DCONS_CLEAN_LOOP_T_2    \n\t"
		"                                \n\t"
		"                                \n\t"
		".DMAIN_LOOP2_T_2:                \n\t" // main loop
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)		
		"add    %3, %3, %0               \n\t" // next band
		"vmla.f32  q8, q4, q0           \n\t"
		"pld    [%3, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q9, q5, q0           \n\t"
		"sub    r0, r0, #2               \n\t" // iter++
		"vld1.64   {d8, d9, d10, d11}, [%3:128]!   \n\t" // load A0 to registers
		"vld1.64   {d0, d1}, [%4:128]!   \n\t" // load x to registers
		"                                \n\t"
		"add    %3, %3, %0               \n\t" // next band
		"vmla.f32  q8, q4, q1           \n\t"
		"pld    [%3, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q9, q5, q1           \n\t"
		"vld1.64   {d8, d9, d10, d11}, [%3:128]!   \n\t" // load A0 to registers
		"cmp    r0, #1                   \n\t" // next iter?
		"vld1.64   {d2, d3}, [%4:128]!   \n\t" // load x to registers
#endif		
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)	
		"pld    [%3, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q8, q4, q0           \n\t"
		"vmla.f32  q9, q5, q0           \n\t"
		"add    %3, %3, %0               \n\t" // next band
		"sub    r0, r0, #2               \n\t" // iter++
		"vld1.64   {d8, d9, d10, d11}, [%3:128]!   \n\t" // load A0 to registers
		"vld1.64   {d0, d1}, [%4:128]!   \n\t" // load x to registers
		"                                \n\t"
		"pld    [%3, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q8, q4, q1           \n\t"
		"vmla.f32  q9, q5, q1           \n\t"
		"add    %3, %3, %0               \n\t" // next band
		"cmp    r0, #1                   \n\t" // next iter?
		"vld1.64   {d8, d9, d10, d11}, [%3:128]!   \n\t" // load A0 to registers
		"vld1.64   {d2, d3}, [%4:128]!   \n\t" // load x to registers
#endif		
		"                                \n\t"
		"bgt    .DMAIN_LOOP2_T_2          \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"blt    .DCONS_CLEAN_LOOP_T_2    \n\t"
		"                                \n\t"
		"                                \n\t"
		".DMAIN_LOOP_T_2:                \n\t" // main loop
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)		
		"add    %3, %3, %0               \n\t" // next band
		"vmla.f32  q8, q4, q0           \n\t"
		"pld    [%3, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q9, q5, q0           \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"vld1.64   {d8, d9, d10, d11}, [%3:128]!   \n\t" // load A0 to registers
		"cmp    r0, #0                   \n\t" // next iter?
		"vmov   q0, q1                   \n\t"
		"vld1.64   {d2, d3}, [%4:128]!   \n\t" // load x to registers
#endif		
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)	
		"pld    [%3, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q8, q4, q0           \n\t"
		"vmla.f32  q9, q5, q0           \n\t"
		"vmov   q0, q1                   \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"add    %3, %3, %0               \n\t" // next band
		"cmp    r0, #0                   \n\t" // next iter?
		"vld1.64   {d8, d9, d10, d11}, [%3:128]!   \n\t" // load A0 to registers
		"vld1.64   {d2, d3}, [%4:128]!   \n\t" // load x to registers
#endif		
		"                                \n\t"
		"bgt    .DMAIN_LOOP_T_2          \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONS_CLEAN_LOOP_T_2:          \n\t" // main loop
		"                                \n\t"
		"mov    r0, %2                   \n\t" // k_left
		"cmp    r0, #0                   \n\t"
		"                                \n\t"
		"ble    .DPOSTACCUM_T_2          \n\t"
		"                                \n\t"
		"vmov   s3, s8                   \n\t"
		"cmp    r0, #2                   \n\t"
		"vmoveq s2, s8                   \n\t"
		"vmovlt s1, s8                   \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q8, q4, q0            \n\t"
		"vmla.f32  q9, q5, q0            \n\t"
		"                                \n\t"
		"                                \n\t"
		".DPOSTACCUM_T_2:                \n\t"
		"                                \n\t"
		"                                \n\t"
		"pld    [r2, #0]                 \n\t" // prefetch A to L1
		"                                \n\t"
		"                                \n\t"
		"mov    r0, r3                   \n\t" // load address of x[0]
		"vld1.64   {d2}, [r0:64]         \n\t" // load x[0]
		"                                \n\t"
		"                                \n\t"
		"vld1.64   {d8, d9, d10, d11}, [r2:128]!   \n\t" // load A to registers
		"                                \n\t"
		"                                \n\t"
		// top trinagle
		"vpadd.f32 d4, d16, d17          \n\t"
		"vpadd.f32 d5, d18, d19          \n\t"
		"                                \n\t"
		"vpadd.f32 d0, d4, d5            \n\t"
		"                                \n\t"
		"vsub.f32  d0, d2, d0            \n\t"
		"                                \n\t"
		"vmul.f32  s1, s21, s1           \n\t"
		"                                \n\t"
		"vmls.f32  s0, s17, s1           \n\t"
		"vmul.f32  s0, s16, s0           \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"mov    r0, r3                   \n\t" // load address of x[0] */
		"vst1.64   {d0}, [r0:64]         \n\t" // store x[0]
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
		  "r" (incA),		// %0
		  "r" (k_iter),		// %1
		  "r" (k_left),		// %2
		  "r" (A),			// %3
		  "r" (x)			// %4
		: // register clobber list
		  "r0", "r1", "r2", "r3",
		  "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",
		  "d8", "d9", "d10", "d11", "d12", "d13", "d14", "d15",
		  "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23",
		  "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31",
		  "memory"
	);

	}



void kernel_strsv_t_1_lib4(int kmax, float *A, int sda, float *x)
	{

	if(kmax<=0) 
		return;
	
	const int bs = 4;

	__builtin_prefetch( A );

	int incA = bs*(sda-1)*sizeof(float);

	kmax -= 4;
	int k_iter = kmax/4;
	int k_left = kmax%4;

	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"add    r1, %0, #16              \n\t"
		"                                \n\t"
		"pld    [%3, r1]                 \n\t" // prefetch A1 to L1
		"                                \n\t"
		"                                \n\t"
		"vldr   d16, .DZERO_T_1          \n\t" // load zero double
		"vldr   d17, .DZERO_T_1+8        \n\t" // load zero double
		"vmov   q2, q8                   \n\t" // zero vector
		"                                \n\t"
		"                                \n\t"
		"mov    r2, %3                   \n\t" // backup A
		"mov    r3, %4                   \n\t" // backup x
		"                                \n\t"
		"                                \n\t"
		// clean up at the beginning
		"vld1.64   {d0, d1}, [%4:128]!   \n\t" // load x to registers
		"vld1.64   {d8, d9}, [%3:128]!   \n\t" // load A0 to registers
		"                                \n\t"
		"vmov      s0, s8                \n\t" // zero vector
		"                                \n\t"
		"vmla.f32  q8, q4, q0            \n\t"
		"                                \n\t"
		"                                \n\t"
		"add    %3, %3, %0               \n\t" // to next block
		"                                \n\t"
		"                                \n\t"
		"mov    r0, %1                   \n\t" // k_loop
		"cmp    r0, #1                   \n\t"
		"                                \n\t"
		"                                \n\t"
		"vld1.64   {d0, d1, d2, d3}, [%4:128]!   \n\t" // load x to registers
		"vld1.64   {d8, d9}, [%3:128]!   \n\t" // load A0 to registers
		"                                \n\t"
		"                                \n\t"
		"beq    .DMAIN_LOOP_T_1          \n\t"
		"blt    .DCONS_CLEAN_LOOP_T_1    \n\t"
		"                                \n\t"
		"                                \n\t"
		".DMAIN_LOOP2_T_1:                \n\t" // main loop
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)		
		"add    %3, %3, %0               \n\t" // next band
		"vmla.f32  q8, q4, q0           \n\t"
		"pld    [%3, r1]                 \n\t" // prefetch A1 to L1
		"sub    r0, r0, #2               \n\t" // iter++
		"vld1.64   {d8, d9}, [%3:128]!   \n\t" // load A0 to registers
		"vld1.64   {d0, d1}, [%4:128]!   \n\t" // load x to registers
		"                                \n\t"
		"add    %3, %3, %0               \n\t" // next band
		"vmla.f32  q8, q4, q1           \n\t"
		"pld    [%3, r1]                 \n\t" // prefetch A1 to L1
		"vld1.64   {d8, d9}, [%3:128]!   \n\t" // load A0 to registers
		"cmp    r0, #1                   \n\t" // next iter?
		"vld1.64   {d2, d3}, [%4:128]!   \n\t" // load x to registers
#endif		
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)	
		"pld    [%3, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q8, q4, q0           \n\t"
		"add    %3, %3, %0               \n\t" // next band
		"sub    r0, r0, #2               \n\t" // iter++
		"vld1.64   {d8, d9}, [%3:128]!   \n\t" // load A0 to registers
		"vld1.64   {d0, d1}, [%4:128]!   \n\t" // load x to registers
		"                                \n\t"
		"pld    [%3, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q8, q4, q1           \n\t"
		"add    %3, %3, %0               \n\t" // next band
		"cmp    r0, #1                   \n\t" // next iter?
		"vld1.64   {d8, d9}, [%3:128]!   \n\t" // load A0 to registers
		"vld1.64   {d2, d3}, [%4:128]!   \n\t" // load x to registers
#endif		
		"                                \n\t"
		"bgt    .DMAIN_LOOP2_T_1          \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"blt    .DCONS_CLEAN_LOOP_T_1    \n\t"
		"                                \n\t"
		"                                \n\t"
		".DMAIN_LOOP_T_1:                \n\t" // main loop
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)		
		"add    %3, %3, %0               \n\t" // next band
		"vmla.f32  q8, q4, q0           \n\t"
		"pld    [%3, r1]                 \n\t" // prefetch A1 to L1
		"sub    r0, r0, #1               \n\t" // iter++
		"vld1.64   {d8, d9}, [%3:128]!   \n\t" // load A0 to registers
		"cmp    r0, #0                   \n\t" // next iter?
		"vmov   q0, q1                   \n\t"
		"vld1.64   {d2, d3}, [%4:128]!   \n\t" // load x to registers
#endif		
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)	
		"pld    [%3, r1]                 \n\t" // prefetch A1 to L1
		"vmla.f32  q8, q4, q0           \n\t"
		"vmov   q0, q1                   \n\t"
		"sub    r0, r0, #1               \n\t" // iter++
		"add    %3, %3, %0               \n\t" // next band
		"cmp    r0, #0                   \n\t" // next iter?
		"vld1.64   {d8, d9}, [%3:128]!   \n\t" // load A0 to registers
		"vld1.64   {d2, d3}, [%4:128]!   \n\t" // load x to registers
#endif		
		"                                \n\t"
		"bgt    .DMAIN_LOOP_T_1          \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONS_CLEAN_LOOP_T_1:          \n\t" // main loop
		"                                \n\t"
		"mov    r0, %2                   \n\t" // k_left
		"cmp    r0, #0                   \n\t"
		"                                \n\t"
		"ble    .DPOSTACCUM_T_1          \n\t"
		"                                \n\t"
		"vmov   s3, s8                   \n\t"
		"cmp    r0, #2                   \n\t"
		"vmoveq s2, s8                   \n\t"
		"vmovlt s1, s8                   \n\t"
		"                                \n\t"
		"                                \n\t"
		"vmla.f32  q8, q4, q0            \n\t"
		"                                \n\t"
		"                                \n\t"
		".DPOSTACCUM_T_1:                \n\t"
		"                                \n\t"
		"                                \n\t"
		"pld    [r2, #0]                 \n\t" // prefetch A to L1
		"                                \n\t"
		"                                \n\t"
		"mov    r0, r3                   \n\t" // load address of x[0]
		"vldr      s4, [r0, #0]          \n\t" // load x[0]
		"                                \n\t"
		"                                \n\t"
		"vldr      s16, [r2, #0]         \n\t" // load A to registers
		"                                \n\t"
		"                                \n\t"
		// top trinagle
		"vpadd.f32 d4, d16, d17          \n\t"
		"                                \n\t"
		"vadd.f32  s0, s8, s9            \n\t"
		"                                \n\t"
		"vsub.f32  s0, s4, s0            \n\t"
		"                                \n\t"
		"vmul.f32  s0, s16, s0           \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"vstr      s0, [r0, #0]          \n\t" // load x[0]
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
		  "r" (incA),		// %0
		  "r" (k_iter),		// %1
		  "r" (k_left),		// %2
		  "r" (A),			// %3
		  "r" (x)			// %4
		: // register clobber list
		  "r0", "r1", "r2", "r3",
		  "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",
		  "d8", "d9", "d10", "d11", "d12", "d13", "d14", "d15",
		  "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23",
		  "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31",
		  "memory"
	);

	}

