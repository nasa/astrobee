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



void kernel_stran_4_lib4(int kmax, int kna, float *A, int sda, float *C)
	{
	
	// kmax is at least 4 !!!
	
	int k;

	const int bs = 4;
	
	k=0;

	if(kna==0)
		{

		C[0+bs*0] = A[0+bs*0];

		C[0+bs*1] = A[1+bs*0];
		C[1+bs*1] = A[1+bs*1];
		
		C[0+bs*2] = A[2+bs*0];
		C[1+bs*2] = A[2+bs*1];
		C[2+bs*2] = A[2+bs*2];

		C[0+bs*3] = A[3+bs*0];
		C[1+bs*3] = A[3+bs*1];
		C[2+bs*3] = A[3+bs*2];
		C[3+bs*3] = A[3+bs*3];
		
		A += 4*sda;
		C += 4*bs;
		k += 4;
		
		}
	else if(kna==1)
		{
		
		// top 1x1 triangle
		C[0+bs*0] = A[0+bs*0];
		
		A += 1 + bs*(sda-1);
		C += bs;
		k += 1;

		// 4x4
		C[0+bs*0] = A[0+bs*0];
		C[1+bs*0] = A[0+bs*1];
		
		C[0+bs*1] = A[1+bs*0];
		C[1+bs*1] = A[1+bs*1];
		C[2+bs*1] = A[1+bs*2];
		
		C[0+bs*2] = A[2+bs*0];
		C[1+bs*2] = A[2+bs*1];
		C[2+bs*2] = A[2+bs*2];
		C[3+bs*2] = A[2+bs*3];
		
		if(kmax==4)
			return;

		C[0+bs*3] = A[3+bs*0];
		C[1+bs*3] = A[3+bs*1];
		C[2+bs*3] = A[3+bs*2];
		C[3+bs*3] = A[3+bs*3];

		A += bs*sda;
		C += 4*bs;
		k += 4;

		}
	else if(kna==2)
		{

		// top 2x2 triangle
		C[0+bs*0] = A[0+bs*0];

		C[0+bs*1] = A[1+bs*0];
		C[1+bs*1] = A[1+bs*1];

		A += 2 + bs*(sda-1);
		C += 2*bs;
		k += 2;

		C[0+bs*0] = A[0+bs*0];
		C[1+bs*0] = A[0+bs*1];
		C[2+bs*0] = A[0+bs*2];
		
		C[0+bs*1] = A[1+bs*0];
		C[1+bs*1] = A[1+bs*1];
		C[2+bs*1] = A[1+bs*2];
		C[3+bs*1] = A[1+bs*3];
		
		if(kmax==4)
			return;

		C[0+bs*2] = A[2+bs*0];
		C[1+bs*2] = A[2+bs*1];
		C[2+bs*2] = A[2+bs*2];
		C[3+bs*2] = A[2+bs*3];
		
		if(kmax==5)
			return;

		C[0+bs*3] = A[3+bs*0];
		C[1+bs*3] = A[3+bs*1];
		C[2+bs*3] = A[3+bs*2];
		C[3+bs*3] = A[3+bs*3];

		A += bs*sda;
		C += 4*bs;
		k += 4;

		}
	else // if(kna==3)
		{

		// top 1x1 triangle
		C[0+bs*0] = A[0+bs*0];

		// 2x2 square
		C[0+bs*1] = A[1+bs*0];
		C[0+bs*2] = A[2+bs*0];
		C[1+bs*1] = A[1+bs*1];
		C[1+bs*2] = A[2+bs*1];

		// low 1x1 triangle
		C[2+bs*2] = A[2+bs*2];

		A += 3 + bs*(sda-1);
		C += 3*bs;
		k += 3;

		}

	int incA = bs*(sda-4)*sizeof(float);
	int k_iter = (kmax-k)/4;
	int k_left = (kmax-k)%4;

//printf("\n%d %d %d\n", kmax, k, k_iter);

//	k += k_iter*4;

	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"pld    [%2, #0]                 \n\t" // prefetch A1 to L1
#if defined(TARGET_CORTEX_A9)
		"pld    [%2, #32]                 \n\t" // prefetch A1 to L1
#endif		
		"                                \n\t"
		"                                \n\t"
		"add     r3, %1, #64             \n\t"
#if defined(TARGET_CORTEX_A9)
		"add     r4, r3, #32             \n\t"
#endif		
//		"add     r3, r3                  \n\t"
		"                                \n\t"
		"                                \n\t"
		"mov     r0, %0                  \n\t"
		"cmp     r0, #1                  \n\t"
		"                                \n\t"
		"                                \n\t"
//		"mov     r1, %2                  \n\t"
//		"mov     r2, %3                  \n\t"
		"                                \n\t"
		"                                \n\t"
		"ble    .DCONS_LOOP1             \n\t"
		"                                \n\t"
		"                                \n\t"
		".DMAIN_LOOP2:                   \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A15)		
		"pld    [%2, r3]                 \n\t" // prefetch A1 to L1
		"vld4.32 {d0, d2, d4, d6}, [%2:128]! \n\t" // load A to registers
		"vld4.32 {d1, d3, d5, d7}, [%2:128]! \n\t" // load A to registers
//		"vld1.32 {d0, d1, d2, d3}, [%2:128]! \n\t" // load A to registers
//		"vld1.32 {d4, d5, d6, d7}, [%2:128]! \n\t" // load A to registers
//		"vtrn.32 q0, q1                  \n\t"
//		"vtrn.32 q2, q3                  \n\t"
//		"vswp    d1, d4                  \n\t"
//		"vswp    d3, d6                  \n\t"
		"add     %2, %1                  \n\t"
//		"pldw   [%3, #128]                \n\t" // prefetch A1 to L1
		"vst1.32 {d0, d1, d2, d3}, [%3:128]! \n\t" // store C from registers
		"sub    r0, r0, #2               \n\t" // iter++
		"vst1.32 {d4, d5, d6, d7}, [%3:128]! \n\t" // store C from registers
		"                                \n\t"
		"                                \n\t"
		"pld    [%2, r3]                 \n\t" // prefetch A1 to L1
		"vld4.32 {d0, d2, d4, d6}, [%2:128]! \n\t" // load A to registers
		"cmp    r0, #1                   \n\t" // next iter?
		"vld4.32 {d1, d3, d5, d7}, [%2:128]! \n\t" // load A to registers
//		"vld1.32 {d0, d1, d2, d3}, [%2:128]! \n\t" // load A to registers
//		"vld1.32 {d4, d5, d6, d7}, [%2:128]! \n\t" // load A to registers
//		"vtrn.32 q0, q1                  \n\t"
//		"vtrn.32 q2, q3                  \n\t"
//		"vswp    d1, d4                  \n\t"
//		"vswp    d3, d6                  \n\t"
		"add     %2, %1                  \n\t"
//		"vst1.32 {d0, d1, d2, d3}, [%3:128]! \n\t" // store C from registers
//		"vst1.32 {d4, d5, d6, d7}, [%3:128]! \n\t" // store C from registers
//		"pldw   [%3, #128]                \n\t" // prefetch A1 to L1
		"vst1.32 {d0, d1, d2, d3}, [%3:128]! \n\t" // store C from registers
		"vst1.32 {d4, d5, d6, d7}, [%3:128]! \n\t" // store C from registers
#endif		
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7)	
		"pld    [%2, r3]                 \n\t" // prefetch A1 to L1
		"pld    [%2, r4]                 \n\t" // prefetch A1 to L1
//		"pldw   [%3, #64]                \n\t" // prefetch A1 to L1
//		"pldw   [%3, #96]                \n\t" // prefetch A1 to L1
		"vld4.32 {d0, d2, d4, d6}, [%2:128]! \n\t" // load A to registers
		"vld4.32 {d1, d3, d5, d7}, [%2:128]! \n\t" // load A to registers
		"add     %2, %1                  \n\t"
		"sub    r0, r0, #2               \n\t" // iter++
		"vst1.32 {d0, d1, d2, d3}, [%3:128]! \n\t" // store C from registers
		"vst1.32 {d4, d5, d6, d7}, [%3:128]! \n\t" // store C from registers
		"                                \n\t"
		"                                \n\t"
#if defined(TARGET_CORTEX_A9)
		"pld    [%2, r3]                 \n\t" // prefetch A1 to L1
		"pld    [%2, r4]                 \n\t" // prefetch A1 to L1
//		"pldw   [%3, #64]                \n\t" // prefetch A1 to L1
//		"pldw   [%3, #96]                \n\t" // prefetch A1 to L1
#endif
		"vld4.32 {d0, d2, d4, d6}, [%2:128]! \n\t" // load A to registers
		"vld4.32 {d1, d3, d5, d7}, [%2:128]! \n\t" // load A to registers
		"add     %2, %1                  \n\t"
		"cmp    r0, #1                   \n\t" // next iter?
		"vst1.32 {d0, d1, d2, d3}, [%3:128]! \n\t" // store C from registers
		"vst1.32 {d4, d5, d6, d7}, [%3:128]! \n\t" // store C from registers
#endif		
		"                                \n\t"
		"bgt    .DMAIN_LOOP2             \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONS_LOOP1:                   \n\t"
		"                                \n\t"
//		"cmp     r0, #1                  \n\t"
		"blt    .DCONS_LEFT1             \n\t"
		"                                \n\t"
		"vld4.32 {d0, d2, d4, d6}, [%2:128]! \n\t" // load A to registers
		"vld4.32 {d1, d3, d5, d7}, [%2:128]! \n\t" // load A to registers
		"add     %2, %1                  \n\t" // needed!!!
		"vst1.32 {d0, d1, d2, d3}, [%3:128]! \n\t" // store C from registers
		"vst1.32 {d4, d5, d6, d7}, [%3:128]! \n\t" // store C from registers
		"                                \n\t"
		"                                \n\t"
		".DCONS_LEFT1:                   \n\t"
		"                                \n\t"
		"                                \n\t"
		"cmp    %4, #1                   \n\t"
		"                                \n\t"
		"blt    .DEND                    \n\t"
		"bgt    .DCONS_LEFT2             \n\t"
		"                                \n\t"
		// k_left == 1
		"vldr   s0, [%2, #0]             \n\t"
		"vldr   s1, [%2, #16]            \n\t"
		"vldr   s2, [%2, #32]            \n\t"
		"vldr   s3, [%2, #48]            \n\t"
		"vst1.32 {d0, d1}, [%3:128]      \n\t" // store C from registers
		"                                \n\t"
		"b      .DEND                    \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONS_LEFT2:                   \n\t"
		"                                \n\t"
		"cmp    %4, #2                   \n\t"
		"                                \n\t"
		// k_left == 2
		"vldr   d0, [%2, #0]             \n\t"
		"vldr   d2, [%2, #16]            \n\t"
		"vldr   d1, [%2, #32]            \n\t"
		"vldr   d3, [%2, #48]            \n\t"
		"vtrn.32 q0, q1                  \n\t"
		"vst1.32 {d0, d1, d2, d3}, [%3:128]! \n\t" // store C from registers
		"                                \n\t"
		"beq    .DEND                    \n\t"
		"                                \n\t"
		// k_left == 3
		"vldr   s0, [%2, #8]             \n\t"
		"vldr   s1, [%2, #24]            \n\t"
		"vldr   s2, [%2, #40]            \n\t"
		"vldr   s3, [%2, #56]            \n\t"
		"vst1.32 {d0, d1}, [%3:128]      \n\t" // store C from registers
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DEND:                          \n\t"
		"                                \n\t"
		"                                \n\t"
		: // output operands (none)
		: // input operands
		  "r" (k_iter),	// %0
		  "r" (incA),	// %1
		  "r" (A),		// %2
		  "r" (C),		// %3
		  "r" (k_left)	// %4
		: // register clobber list
		  "r0", "r1", "r2", "r3", "r4",
		  "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",
//		  "d8", "d9", "d10", "d11", "d12", "d13", "d14", "d15",
//		  "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23",
//		  "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31",
		  "memory"
	);

/*printf("\n%d %d %d\n", kmax, k, k_iter);*/
/*s_print_pmat(4*k_iter, 4, 4, A, sda);*/
/*s_print_mat(4, 4*k_iter, C, 4);*/
/*exit(1);*/
	
	}



void corner_stran_3_lib4(int kna, float *A, int sda, float *C)
	{

	const int bs = 4;
	
	if(kna==0)
		{
		
		C[0+bs*0] = A[0+bs*0];
		
		C[0+bs*1] = A[1+bs*0];
		C[1+bs*1] = A[1+bs*1];
		
		C[0+bs*2] = A[2+bs*0];
		C[1+bs*2] = A[2+bs*1];
		C[2+bs*2] = A[2+bs*2];

		}
	else if(kna==1)
		{
		
		C[0+bs*0] = A[0+bs*0];
		
		A += 1 + bs*(sda-1);
		C += bs;

		C[0+bs*0] = A[0+bs*0];
		C[1+bs*0] = A[0+bs*1];
		
		C[0+bs*1] = A[1+bs*0];
		C[1+bs*1] = A[1+bs*1];
		C[2+bs*1] = A[1+bs*2];

		}
	else if(kna==2)
		{

		C[0+bs*0] = A[0+bs*0];
		
		C[0+bs*1] = A[1+bs*0];
		C[1+bs*1] = A[1+bs*1];

		A += 2 + bs*(sda-1);
		C += 2*bs;

		C[0+bs*0] = A[0+bs*0];
		C[1+bs*0] = A[0+bs*1];
		C[2+bs*0] = A[0+bs*2];

		}
	else // if(kna==3)
		{

		C[0+bs*0] = A[0+bs*0];
		
		C[0+bs*1] = A[1+bs*0];
		C[1+bs*1] = A[1+bs*1];
		
		C[0+bs*2] = A[2+bs*0];
		C[1+bs*2] = A[2+bs*1];
		C[2+bs*2] = A[2+bs*2];

		}

	}



void corner_stran_2_lib4(int kna, float *A, int sda, float *C)
	{

	const int bs = 4;
	
	if(kna==1)
		{

		C[0+bs*0] = A[0+bs*0];
		
		A += 1 + bs*(sda-1);
		C += bs;

		C[0+bs*0] = A[0+bs*0];
		C[1+bs*0] = A[0+bs*1];

		}
	else // if(kna==3)
		{
		
		C[0+bs*0] = A[0+bs*0];
		
		C[0+bs*1] = A[1+bs*0];
		C[1+bs*1] = A[1+bs*1];

		}

	}




