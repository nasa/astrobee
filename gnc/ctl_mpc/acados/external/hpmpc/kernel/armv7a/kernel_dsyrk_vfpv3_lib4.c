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

#include <math.h>

#include "../../include/block_size.h"



// normal-transposed, 4x4 with data packed in 4
// prefetch optimized for Cortex-A9 (cache line is 32 bytes, while A15 is 64 bytes)
/*void kernel_dgemm_pp_nt_4x4_lib4(int kmax, double *A, double *B, double *C, double *D, int alg)*/
void kernel_dsyrk_nt_4x4_vs_lib4(int km, int kn, int kmax, double *A, double *B, double *C, double *D, int alg)
	{
	
//	if(kmax<=0)
//		return;
	
	if(kmax<=0) // add & store
		{

		const int bs = 4;

		double
			c_00=0, 
			c_10=0, c_11=0, 
			c_20=0, c_21=0, c_22=0, 
			c_30=0, c_31=0, c_32=0, c_33=0;
		
		if(alg==0) // C = A * B'
			{
			goto store;
			}
		else 
			{
			c_00 = C[0+bs*0];
			c_10 = C[1+bs*0];
			c_20 = C[2+bs*0];
			c_30 = C[3+bs*0];
			
			c_11 = C[1+bs*1];
			c_21 = C[2+bs*1];
			c_31 = C[3+bs*1];
			
			c_22 = C[2+bs*2];
			c_32 = C[3+bs*2];
			
			c_33 = C[3+bs*3];
			
			goto store;
			}
		
		store:
		if(km>=4)
			{
			D[0+bs*0] = c_00;
			D[1+bs*0] = c_10;
			D[2+bs*0] = c_20;
			D[3+bs*0] = c_30;

			D[1+bs*1] = c_11;
			D[2+bs*1] = c_21;
			D[3+bs*1] = c_31;

			D[2+bs*2] = c_22;
			D[3+bs*2] = c_32;

			if(kn>=4)
				D[3+bs*3] = c_33;
			}
		else
			{
			D[0+bs*0] = c_00;
			D[1+bs*0] = c_10;
			D[2+bs*0] = c_20;

			D[1+bs*1] = c_11;
			D[2+bs*1] = c_21;

			D[2+bs*2] = c_22;
			}

		return;

		}

	
	__builtin_prefetch( A );
	__builtin_prefetch( B );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A+4 );
	__builtin_prefetch( B+4 );
#endif

	int ki_add = kmax/4;
	int kl_add = kmax%4;

	const int bs = D_MR;//4;

	__builtin_prefetch( A+8 );
	__builtin_prefetch( B+8 );
#if defined(TARGET_CORTEX_A9)
	__builtin_prefetch( A+12 );
	__builtin_prefetch( B+12 );
#endif

//	printf("\n%d %d %d\n", kmax, k_iter, k_left);

	__asm__ volatile
	(
		"                                \n\t"
		"mov    r3, %0                   \n\t" // k_iter
		"                                \n\t"
		"                                \n\t"
		"fldd   d16, [%2, #0]            \n\t" // prefetch A_even
		"fldd   d17, [%2, #8]            \n\t"
		"fldd   d18, [%2, #16]           \n\t"
		"fldd   d19, [%2, #24]           \n\t"
		"                                \n\t"
		"fldd   d20, [%3, #0]            \n\t" // prefetch B_even
		"fldd   d21, [%3, #8]            \n\t"
		"fldd   d22, [%3, #16]           \n\t"
		"fldd   d23, [%3, #24]           \n\t"
		"                                \n\t"
		"cmp    r3, #0                   \n\t"
		"                                \n\t"
		"fldd   d24, [%2, #32]           \n\t" // prefetch A_odd
		"fldd   d25, [%2, #40]           \n\t"
		"fldd   d26, [%2, #48]           \n\t"
		"fldd   d27, [%2, #56]           \n\t"
		"                                \n\t"
		"fldd   d28, [%3, #32]           \n\t" // prefetch B_odd
		"fldd   d29, [%3, #40]           \n\t"
		"fldd   d30, [%3, #48]           \n\t"
		"fldd   d31, [%3, #56]           \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"fldd   d0, .DOUBLEZERO          \n\t" // load zero double
		"fcpyd  d1, d0                   \n\t"
		"fcpyd  d2, d0                   \n\t"
		"fcpyd  d3, d0                   \n\t"
		"fcpyd  d4, d0                   \n\t"
		"fcpyd  d5, d0                   \n\t"
		"fcpyd  d6, d0                   \n\t"
		"fcpyd  d7, d0                   \n\t"
		"fcpyd  d8, d0                   \n\t"
		"fcpyd  d9, d0                   \n\t"
		"fcpyd  d10, d0                  \n\t"
		"fcpyd  d11, d0                  \n\t"
		"fcpyd  d12, d0                  \n\t"
		"fcpyd  d13, d0                  \n\t"
		"fcpyd  d14, d0                  \n\t"
		"fcpyd  d15, d0                  \n\t"
		"                                \n\t"
		"                                \n\t"
		"ble    .DCONSIDERADD            \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"b      .DENDZERO                \n\t"
		".align 3                        \n\t"
		".DOUBLEZERO:                    \n\t" // zero double word
		".word  0                        \n\t"
		".word  0                        \n\t"
		".DENDZERO:                      \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DLOOPADD:                      \n\t" // main loop
		"                                \n\t"
		"                                \n\t"
		"pld    [%2, #128]               \n\t"
		"pld    [%3, #128]               \n\t"
		"                                \n\t"
		"                                \n\t"
		"fmacd  d0, d16, d20             \n\t"
		"fldd   d16, [%2, #64]           \n\t" // prefetch A_even
		"fmacd  d1, d17, d20             \n\t"
		"fmacd  d2, d18, d20             \n\t"
		"fmacd  d3, d19, d20             \n\t"
		"fldd   d20, [%3, #64]           \n\t" // prefetch B_even
		"                                \n\t"
		"fmacd  d5, d17, d21             \n\t"
		"fldd   d17, [%2, #72]           \n\t"
		"fmacd  d6, d18, d21             \n\t"
		"fmacd  d7, d19, d21             \n\t"
		"fldd   d21, [%3, #72]           \n\t"
		"                                \n\t"
		"fmacd  d10, d18, d22            \n\t"
		"fldd   d18, [%2, #80]           \n\t"
		"fmacd  d11, d19, d22            \n\t"
		"fldd   d22, [%3, #80]           \n\t"
		"                                \n\t"
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
		"fldd   d24, [%2, #96]           \n\t" // prefetch A_odd
		"fmacd  d3, d27, d28             \n\t"
		"fldd   d28, [%3, #96]           \n\t" // prefetch B_odd
		"                                \n\t"
		"fmacd  d5, d25, d29             \n\t"
		"fldd   d25, [%2, #104]          \n\t"
		"fmacd  d6, d26, d29             \n\t"
		"fmacd  d7, d27, d29             \n\t"
		"fldd   d29, [%3, #104]          \n\t"
		"                                \n\t"
		"fmacd  d10, d26, d30            \n\t"
		"fldd   d26, [%2, #112]          \n\t"
		"fmacd  d11, d27, d30            \n\t"
		"fldd   d30, [%3, #112]          \n\t"
		"                                \n\t"
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
		"fldd   d16, [%2, #128]          \n\t" // prefetch A_even
		"fmacd  d3, d19, d20             \n\t"
		"fldd   d20, [%3, #128]          \n\t" // prefetch B_even
		"                                \n\t"
		"fmacd  d5, d17, d21             \n\t"
		"fldd   d17, [%2, #136]          \n\t"
		"fmacd  d6, d18, d21             \n\t"
		"fmacd  d7, d19, d21             \n\t"
		"fldd   d21, [%3, #136]          \n\t"
		"                                \n\t"
		"fmacd  d10, d18, d22            \n\t"
		"fldd   d18, [%2, #144]          \n\t"
		"fmacd  d11, d19, d22            \n\t"
		"fldd   d22, [%3, #144]          \n\t"
		"                                \n\t"
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
		"fmacd  d5, d25, d29             \n\t"
		"fldd   d24, [%2, #32]           \n\t" // prefetch A_odd
		"fmacd  d6, d26, d29             \n\t"
		"fldd   d25, [%2, #40]           \n\t"
		"fmacd  d7, d27, d29             \n\t"
		"fldd   d29, [%3, #40]           \n\t"
		"                                \n\t"
		"fmacd  d10, d26, d30            \n\t"
		"fldd   d26, [%2, #48]           \n\t"
		"fmacd  d11, d27, d30            \n\t"
		"fldd   d30, [%3, #48]           \n\t"
		"                                \n\t"
		"fmacd  d15, d27, d31            \n\t"
		"fldd   d31, [%3, #56]           \n\t"
		"fldd   d27, [%2, #56]           \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"bgt    .DLOOPADD                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERADD:                  \n\t" // consider left
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"mov    r3, %1                   \n\t" // k_left
		"cmp    r3, #0                   \n\t"
		"ble    .DPOSTACC                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DLOOPLEFT:                     \n\t" // clean up loop
		"                                \n\t"
		"sub    r3, r3, #1               \n\t"
		"                                \n\t"
		"fmacd  d0, d16, d20             \n\t"
		"fldd   d16, [%2, #32]           \n\t" // prefetch A_even
		"fmacd  d1, d17, d20             \n\t"
		"fmacd  d2, d18, d20             \n\t"
		"fmacd  d3, d19, d20             \n\t"
		"fldd   d20, [%3, #32]           \n\t" // prefetch B_even
		"                                \n\t"
		"fmacd  d5, d17, d21             \n\t"
		"fldd   d17, [%2, #40]           \n\t"
		"fmacd  d6, d18, d21             \n\t"
		"fmacd  d7, d19, d21             \n\t"
		"fldd   d21, [%3, #40]           \n\t"
		"                                \n\t"
		"fmacd  d10, d18, d22            \n\t"
		"fldd   d18, [%2, #48]           \n\t"
		"fmacd  d11, d19, d22            \n\t"
		"fldd   d22, [%3, #48]           \n\t"
		"                                \n\t"
		"cmp    r3, #0                   \n\t"
		"                                \n\t"
		"fmacd  d15, d19, d23            \n\t"
		"fldd   d19, [%2, #56]           \n\t"
		"add    %2, %2, #32              \n\t"
		"fldd   d23, [%3, #56]           \n\t"
		"add    %3, %3, #32              \n\t"
		"                                \n\t"
		"                                \n\t"
		"bgt    .DLOOPLEFT               \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DPOSTACC:                      \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"pldw   [%6, #0]                 \n\t" // prefetch C*/
#if defined(TARGET_CORTEX_A9)
/*		"pldw   [%6, #32]                \n\t"*/
#endif
/*		"pldw   [%6, #64]                \n\t"*/
#if defined(TARGET_CORTEX_A9)
/*		"pldw   [%6, #96]                \n\t"*/
#endif
//		"pld    [%6, #0]                 \n\t" // prefetch C
//		"pld    [%6, #32]                \n\t"
//		"pld    [%6, #64]                \n\t"
//		"pld    [%6, #96]                \n\t"

//		"ldr    r3, %5                   \n\t" // alg
		"mov    r3, %5                   \n\t" // alg
		"cmp    r3, #0                   \n\t"
		"beq    .D0                      \n\t" // if alg==0, jump
		"                                \n\t"
		"cmp    r3, #1                   \n\t"
		"                                \n\t"
		"fldd   d16, [%4, #0]            \n\t" // load C elements
		"fldd   d17, [%4, #8]            \n\t"
		"fldd   d18, [%4, #16]           \n\t"
		"fldd   d19, [%4, #24]           \n\t"
		"                                \n\t"
//		"fldd   d20, [%4, #32]           \n\t"
		"fldd   d21, [%4, #40]           \n\t"
		"fldd   d22, [%4, #48]           \n\t"
		"fldd   d23, [%4, #56]           \n\t"
		"                                \n\t"
//		"fldd   d24, [%4, #64]           \n\t"
//		"fldd   d25, [%4, #72]           \n\t"
		"fldd   d26, [%4, #80]           \n\t"
		"fldd   d27, [%4, #88]           \n\t"
		"                                \n\t"
//		"fldd   d28, [%4, #96]           \n\t"
//		"fldd   d29, [%4, #104]          \n\t"
//		"fldd   d30, [%4, #112]          \n\t"
		"fldd   d31, [%4, #120]          \n\t"
		"                                \n\t"
		"beq    .D1                      \n\t" // if alg==1, jump
		"                                \n\t"
		"                                \n\t"// alg==-1
		"fsubd  d0, d16, d0              \n\t"
		"fsubd  d1, d17, d1              \n\t"
		"fsubd  d2, d18, d2              \n\t"
		"fsubd  d3, d19, d3              \n\t"
		"                                \n\t"
//		"fsubd  d4, d20, d4              \n\t"
		"fsubd  d5, d21, d5              \n\t"
		"fsubd  d6, d22, d6              \n\t"
		"fsubd  d7, d23, d7              \n\t"
		"                                \n\t"
//		"fsubd  d8, d24, d8              \n\t"
//		"fsubd  d9, d25, d9              \n\t"
		"fsubd  d10, d26, d10            \n\t"
		"fsubd  d11, d27, d11            \n\t"
		"                                \n\t"
//		"fsubd  d12, d28, d12            \n\t"
//		"fsubd  d13, d29, d13            \n\t"
//		"fsubd  d14, d30, d14            \n\t"
		"fsubd  d15, d31, d15            \n\t"
		"                                \n\t"
		"b      .D0                      \n\t" // jump to end
		"                                \n\t"
		"                                \n\t"
		".D1:                            \n\t" // alg==1
		"                                \n\t"
		"faddd  d0, d0, d16              \n\t"
		"faddd  d1, d1, d17              \n\t"
		"faddd  d2, d2, d18              \n\t"
		"faddd  d3, d3, d19              \n\t"
		"                                \n\t"
//		"faddd  d4, d4, d20              \n\t"
		"faddd  d5, d5, d21              \n\t"
		"faddd  d6, d6, d22              \n\t"
		"faddd  d7, d7, d23              \n\t"
		"                                \n\t"
//		"faddd  d8, d8, d24              \n\t"
//		"faddd  d9, d9, d25              \n\t"
		"faddd  d10, d10, d26            \n\t"
		"faddd  d11, d11, d27            \n\t"
		"                                \n\t"
//		"faddd  d12, d12, d28            \n\t"
//		"faddd  d13, d13, d29            \n\t"
//		"faddd  d14, d14, d30            \n\t"
		"faddd  d15, d15, d31            \n\t"
		"                                \n\t"
		"                                \n\t"
		".D0:                            \n\t" // alg==0
		"                                \n\t"
//		"mov    r3, %7                   \n\t" // km
//		"cmp    r3, #4                   \n\t"
		"cmp    %7, #4                   \n\t"
		"blt    .DKM3                    \n\t" // if km<4, jump
		"                                \n\t"
		"fstd   d0, [%6, #0]             \n\t" // store result
		"fstd   d1, [%6, #8]             \n\t"
		"fstd   d2, [%6, #16]            \n\t"
		"fstd   d3, [%6, #24]            \n\t"
		"                                \n\t"
//		"mov    r3, %8                   \n\t" // km
		"                                \n\t"
//		"fstd   d4, [%6, #32]            \n\t"
		"fstd   d5, [%6, #40]            \n\t"
		"fstd   d6, [%6, #48]            \n\t"
		"fstd   d7, [%6, #56]            \n\t"
		"                                \n\t"
//		"cmp    r3, #4                   \n\t"
		"cmp    %8, #4                   \n\t"
		"                                \n\t"
//		"fstd   d8, [%6, #64]            \n\t"
//		"fstd   d9, [%6, #72]            \n\t"
		"fstd   d10, [%6, #80]           \n\t"
		"fstd   d11, [%6, #88]           \n\t"
		"                                \n\t"
		"blt    .DEND                    \n\t" // if kn<4, jump
		"                                \n\t"
//		"fstd   d12, [%6, #96]           \n\t"
//		"fstd   d13, [%6, #104]          \n\t"
//		"fstd   d14, [%6, #112]          \n\t"
		"fstd   d15, [%6, #120]          \n\t"
		"                                \n\t"
		"b      .DEND                    \n\t" // return
		"                                \n\t"
		".DKM3:                          \n\t"
		"                                \n\t"
		"fstd   d0, [%6, #0]             \n\t" // store result
		"fstd   d1, [%6, #8]             \n\t"
		"fstd   d2, [%6, #16]            \n\t"
//		"fstd   d3, [%6, #24]            \n\t"
		"                                \n\t"
//		"fstd   d4, [%6, #32]            \n\t"
		"fstd   d5, [%6, #40]            \n\t"
		"fstd   d6, [%6, #48]            \n\t"
//		"fstd   d7, [%6, #56]            \n\t"
		"                                \n\t"
//		"fstd   d8, [%6, #64]            \n\t"
//		"fstd   d9, [%6, #72]            \n\t"
		"fstd   d10, [%6, #80]           \n\t"
//		"fstd   d11, [%6, #88]           \n\t"
		"                                \n\t"
//		"fstd   d12, [%6, #96]           \n\t"
//		"fstd   d13, [%6, #104]          \n\t"
//		"fstd   d14, [%6, #112]          \n\t"
//		"fstd   d15, [%6, #120]          \n\t"
		"                                \n\t"
		"                                \n\t"
		".DEND:                          \n\t"
		"                                \n\t"
		"                                \n\t"
		: // output operands (none)
		: // input operands
		  "r" (ki_add),		// %0
		  "r" (kl_add),		// %1
		  "r" (A),			// %2
		  "r" (B),			// %3
		  "r" (C),			// %4
		  "r" (alg),		// %5
		  "r" (D),			// %6
		  "r" (km),			// %7
		  "r" (kn)			// %8
		: // register clobber list
		  "r3",
		  "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",
		  "d8", "d9", "d10", "d11", "d12", "d13", "d14", "d15",
		  "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23",
		  "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31",
		  "memory"
	);
}



void kernel_dsyrk_nt_4x2_vs_lib4(int km, int kn, int kmax, double *A, double *B, double *C, double *D, int alg)
	{

//	if(kmax<=0)
//		return;
	
	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1,
		c_00=0, 
		c_10=0, c_11=0, 
		c_20=0, c_21=0,  
		c_30=0, c_31=0;
		
	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		b_0 = B[0+bs*1];
		b_1 = B[1+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		b_0 = B[0+bs*2];
		b_1 = B[1+bs*2];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
		
		b_0 = B[0+bs*3];
		b_1 = B[1+bs*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;
		
		
		A += 16;
		B += 16;

		}
	for(; k<kmax; k++)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		A += 4;
		B += 4;

		}
	double
		d_00,
		d_10, d_11,
		d_20, d_21,
		d_30, d_31;
	
	if(alg==0) // C = A * B'
		{
		goto store;
		}
	else 
		{
		d_00 = C[0+bs*0];
		d_10 = C[1+bs*0];
		d_20 = C[2+bs*0];
		d_30 = C[3+bs*0];
		
		d_11 = C[1+bs*1];
		d_21 = C[2+bs*1];
		d_31 = C[3+bs*1];
		
		if(alg==1) // C += A * B'
			{
			c_00 = d_00 + c_00;
			c_10 = d_10 + c_10;
			c_20 = d_20 + c_20;
			c_30 = d_30 + c_30;

			c_11 = d_11 + c_11;
			c_21 = d_21 + c_21;
			c_31 = d_31 + c_31;
			}
		else // C -= A * B'
			{
			c_00 = d_00 - c_00;
			c_10 = d_10 - c_10;
			c_20 = d_20 - c_20;
			c_30 = d_30 - c_30;

			c_11 = d_11 - c_11;
			c_21 = d_21 - c_21;
			c_31 = d_31 - c_31;
			}

		goto store;
		}

	store:
	if(km>=4)
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;
		D[2+bs*0] = c_20;
		D[3+bs*0] = c_30;

		if(kn>=2)
			{
			D[1+bs*1] = c_11;
			D[2+bs*1] = c_21;
			D[3+bs*1] = c_31;
			}
		}
	else
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;
		D[2+bs*0] = c_20;

		if(kn>=2)
			{
			D[1+bs*1] = c_11;
			D[2+bs*1] = c_21;
			}
		}

	}



void kernel_dsyrk_nt_2x2_vs_lib4(int km, int kn, int kmax, double *A, double *B, double *C, double *D, int alg)
	{

//	if(kmax<=0)
//		return;
	
	const int bs = 4;

	int k;

	double
		a_0, a_1,
		b_0, b_1,
		c_00=0, 
		c_10=0, c_11=0;
		
	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_11 += a_1 * b_1;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		
		b_0 = B[0+bs*1];
		b_1 = B[1+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_11 += a_1 * b_1;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		
		b_0 = B[0+bs*2];
		b_1 = B[1+bs*2];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_11 += a_1 * b_1;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		
		b_0 = B[0+bs*3];
		b_1 = B[1+bs*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_11 += a_1 * b_1;
		
		
		A += 16;
		B += 16;

		}
	for(; k<kmax; k++)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_11 += a_1 * b_1;


		A += 4;
		B += 4;

		}
	
	double
		d_00,
		d_10, d_11;
	
	if(alg==0) // C = A * B'
		{
		goto store;
		}
	else 
		{
		d_00 = C[0+bs*0];
		d_10 = C[1+bs*0];
		
		d_11 = C[1+bs*1];
		
		if(alg==1) // C += A * B'
			{
			c_00 = d_00 + c_00;
			c_10 = d_10 + c_10;

			c_11 = d_11 + c_11;
			}
		else // C -= A * B'
			{
			c_00 = d_00 - c_00;
			c_10 = d_10 - c_10;

			c_11 = d_11 - c_11;
			}

		goto store;
		}

	store:
	if(km>=2)
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;

		if(kn>=2)
			D[1+bs*1] = c_11;
		}
	else
		{
		D[0+bs*0] = c_00;
		}

	}



void kernel_dsyrk_nn_4x4_lib4(int kmax, double *A, double *B, int sdb, double *C, double *D, int alg)
	{

//	if(kmax<=0)
//		return;

	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, 
		c_10=0, c_11=0, 
		c_20=0, c_21=0, c_22=0, 
		c_30=0, c_31=0, c_32=0, c_33=0;
		
	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[0+bs*1];
		b_2 = B[0+bs*2];
		b_3 = B[0+bs*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_33 += a_3 * b_3;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		b_0 = B[1+bs*0];
		b_1 = B[1+bs*1];
		b_2 = B[1+bs*2];
		b_3 = B[1+bs*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_33 += a_3 * b_3;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		b_0 = B[2+bs*0];
		b_1 = B[2+bs*1];
		b_2 = B[2+bs*2];
		b_3 = B[2+bs*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_33 += a_3 * b_3;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
		
		b_0 = B[3+bs*0];
		b_1 = B[3+bs*1];
		b_2 = B[3+bs*2];
		b_3 = B[3+bs*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_33 += a_3 * b_3;
		
		
		A += 16;
		B += 4*sdb;

		}
	for(; k<kmax; k++)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[0+bs*1];
		b_2 = B[0+bs*2];
		b_3 = B[0+bs*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_33 += a_3 * b_3;


		A += 4;
		B += 1;

		}

	double
		d_00,
		d_10, d_11,
		d_20, d_21, d_22,
		d_30, d_31, d_32, d_33;
	
	if(alg==0) // C = A * B'
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;
		D[2+bs*0] = c_20;
		D[3+bs*0] = c_30;

		D[1+bs*1] = c_11;
		D[2+bs*1] = c_21;
		D[3+bs*1] = c_31;

		D[2+bs*2] = c_22;
		D[3+bs*2] = c_32;

		D[3+bs*3] = c_33;
		}
	else 
		{
		d_00 = C[0+bs*0];
		d_10 = C[1+bs*0];
		d_20 = C[2+bs*0];
		d_30 = C[3+bs*0];
		
		d_11 = C[1+bs*1];
		d_21 = C[2+bs*1];
		d_31 = C[3+bs*1];
		
		d_22 = C[2+bs*2];
		d_32 = C[3+bs*2];
		
		d_33 = C[3+bs*3];
		
		if(alg==1) // C += A * B'
			{
			d_00 += c_00;
			d_10 += c_10;
			d_20 += c_20;
			d_30 += c_30;

			d_11 += c_11;
			d_21 += c_21;
			d_31 += c_31;

			d_22 += c_22;
			d_32 += c_32;

			d_33 += c_33;
			}
		else // C -= A * B'
			{
			d_00 -= c_00;
			d_10 -= c_10;
			d_20 -= c_20;
			d_30 -= c_30;

			d_11 -= c_11;
			d_21 -= c_21;
			d_31 -= c_31;

			d_22 -= c_22;
			d_32 -= c_32;

			d_33 -= c_33;
			}

		D[0+bs*0] = d_00;
		D[1+bs*0] = d_10;
		D[2+bs*0] = d_20;
		D[3+bs*0] = d_30;

		D[1+bs*1] = d_11;
		D[2+bs*1] = d_21;
		D[3+bs*1] = d_31;

		D[2+bs*2] = d_22;
		D[3+bs*2] = d_32;

		D[3+bs*3] = d_33;
		}
	
	}



void kernel_dsyrk_nn_4x2_lib4(int kmax, double *A, double *B, int sdb, double *C, double *D, int alg)
	{

//	if(kmax<=0)
//		return;

	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1,
		c_00=0, 
		c_10=0, c_11=0, 
		c_20=0, c_21=0,  
		c_30=0, c_31=0;
		
	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[0+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		a_2 = A[2+bs*1];
		a_3 = A[3+bs*1];
		
		b_0 = B[1+bs*0];
		b_1 = B[1+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		a_2 = A[2+bs*2];
		a_3 = A[3+bs*2];
		
		b_0 = B[2+bs*0];
		b_1 = B[2+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		a_2 = A[2+bs*3];
		a_3 = A[3+bs*3];
		
		b_0 = B[3+bs*0];
		b_1 = B[3+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;
		
		
		A += 16;
		B += 4*sdb;

		}
	for(; k<kmax; k++)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		a_2 = A[2+bs*0];
		a_3 = A[3+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[0+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;
		c_20 += a_2 * b_0;
		c_30 += a_3 * b_0;

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;


		A += 4;
		B += 1;

		}
	double
		d_00,
		d_10, d_11,
		d_20, d_21,
		d_30, d_31;
	
	if(alg==0) // C = A * B'
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;
		D[2+bs*0] = c_20;
		D[3+bs*0] = c_30;

		D[1+bs*1] = c_11;
		D[2+bs*1] = c_21;
		D[3+bs*1] = c_31;
		}
	else 
		{
		d_00 = C[0+bs*0];
		d_10 = C[1+bs*0];
		d_20 = C[2+bs*0];
		d_30 = C[3+bs*0];
		
		d_11 = C[1+bs*1];
		d_21 = C[2+bs*1];
		d_31 = C[3+bs*1];
		
		if(alg==1) // C += A * B'
			{
			d_00 += c_00;
			d_10 += c_10;
			d_20 += c_20;
			d_30 += c_30;

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

			d_11 -= c_11;
			d_21 -= c_21;
			d_31 -= c_31;
			}

		D[0+bs*0] = d_00;
		D[1+bs*0] = d_10;
		D[2+bs*0] = d_20;
		D[3+bs*0] = d_30;

		D[1+bs*1] = d_11;
		D[2+bs*1] = d_21;
		D[3+bs*1] = d_31;
		}

	}



void kernel_dsyrk_nn_2x2_lib4(int kmax, double *A, double *B, int sdb, double *C, double *D, int alg)
	{

//	if(kmax<=0)
//		return;

	const int bs = 4;

	int k;

	double
		a_0, a_1,
		b_0, b_1,
		c_00=0, 
		c_10=0, c_11=0;
		
	for(k=0; k<kmax-3; k+=4)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[0+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_11 += a_1 * b_1;


		a_0 = A[0+bs*1];
		a_1 = A[1+bs*1];
		
		b_0 = B[1+bs*0];
		b_1 = B[1+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_11 += a_1 * b_1;


		a_0 = A[0+bs*2];
		a_1 = A[1+bs*2];
		
		b_0 = B[2+bs*0];
		b_1 = B[2+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_11 += a_1 * b_1;


		a_0 = A[0+bs*3];
		a_1 = A[1+bs*3];
		
		b_0 = B[3+bs*0];
		b_1 = B[3+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_11 += a_1 * b_1;
		
		
		A += 16;
		B += 4*sdb;

		}
	for(; k<kmax; k++)
		{
		
		a_0 = A[0+bs*0];
		a_1 = A[1+bs*0];
		
		b_0 = B[0+bs*0];
		b_1 = B[0+bs*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_11 += a_1 * b_1;


		A += 4;
		B += 1;

		}
	
	double
		d_00,
		d_10, d_11;
	
	if(alg==0) // C = A * B'
		{
		D[0+bs*0] = c_00;
		D[1+bs*0] = c_10;

		D[1+bs*1] = c_11;
		}
	else 
		{
		d_00 = C[0+bs*0];
		d_10 = C[1+bs*0];
		
		d_11 = C[1+bs*1];
		
		if(alg==1) // C += A * B'
			{
			d_00 += c_00;
			d_10 += c_10;

			d_11 += c_11;
			}
		else // C -= A * B'
			{
			d_00 -= c_00;
			d_10 -= c_10;

			d_11 -= c_11;
			}

		D[0+bs*0] = d_00;
		D[1+bs*0] = d_10;

		D[1+bs*1] = d_11;
		}

	}



// Al and Ar are the diagonal of two matrices
void kernel_dsyrk_diag_left_right_4_lib4(int kmax, double *Al, double *Ar, double *B, double *C, double *D, int alg)
	{

	if(kmax<=0)
		return;
	
	// assume kmax to be multiple of 4
	
	const int bs = 4;

	int k;

	double
		a_r,
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_0, c_1, c_2, c_3;
		
	if(alg==-1)
		{
		a_0 = - Al[0];
		a_1 = - Al[1];
		a_2 = - Al[2];
		a_3 = - Al[3];
		}
	else
		{
		a_0 = Al[0];
		a_1 = Al[1];
		a_2 = Al[2];
		a_3 = Al[3];
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-4; k+=4)
			{

			a_r = Ar[0];
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			b_2 = B[2+bs*0];
			b_3 = B[3+bs*0];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;
			c_2 = a_2 * b_2 * a_r;
			c_3 = a_3 * b_3 * a_r;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
	

			a_r = Ar[1];
			
			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			b_2 = B[2+bs*1];
			b_3 = B[3+bs*1];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;
			c_2 = a_2 * b_2 * a_r;
			c_3 = a_3 * b_3 * a_r;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
			D[3+bs*1] = c_3;
	

			a_r = Ar[2];
			
			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			b_2 = B[2+bs*2];
			b_3 = B[3+bs*2];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;
			c_2 = a_2 * b_2 * a_r;
			c_3 = a_3 * b_3 * a_r;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			D[2+bs*2] = c_2;
			D[3+bs*2] = c_3;
	

			a_r = Ar[3];
			
			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			b_2 = B[2+bs*3];
			b_3 = B[3+bs*3];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;
			c_2 = a_2 * b_2 * a_r;
			c_3 = a_3 * b_3 * a_r;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
			D[2+bs*3] = c_2;
			D[3+bs*3] = c_3;
	
			Ar += 4;
			B  += 16;
			C  += 16;
			D  += 16;
			
			}

		a_r = Ar[0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		b_2 = B[2+bs*0];
		b_3 = B[3+bs*0];
		
		c_0 = a_0 * b_0 * a_r;
		c_1 = a_1 * b_1 * a_r;
		c_2 = a_2 * b_2 * a_r;
		c_3 = a_3 * b_3 * a_r;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;
		D[2+bs*0] = c_2;
		D[3+bs*0] = c_3;


		a_r = Ar[1];
		
		b_1 = B[1+bs*1];
		b_2 = B[2+bs*1];
		b_3 = B[3+bs*1];
		
		c_1 = a_1 * b_1 * a_r;
		c_2 = a_2 * b_2 * a_r;
		c_3 = a_3 * b_3 * a_r;

		D[1+bs*1] = c_1;
		D[2+bs*1] = c_2;
		D[3+bs*1] = c_3;


		a_r = Ar[2];
		
		b_2 = B[2+bs*2];
		b_3 = B[3+bs*2];
		
		c_2 = a_2 * b_2 * a_r;
		c_3 = a_3 * b_3 * a_r;

		D[2+bs*2] = c_2;
		D[3+bs*2] = c_3;


		a_r = Ar[3];
		
		b_3 = B[3+bs*3];
		
		c_3 = a_3 * b_3 * a_r;

		D[3+bs*3] = c_3;

		}
	else
		{

		for(k=0; k<kmax-4; k+=4)
			{

			a_r = Ar[0];
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			b_2 = B[2+bs*0];
			b_3 = B[3+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*0] + a_1 * b_1 * a_r;
			c_2 = C[2+bs*0] + a_2 * b_2 * a_r;
			c_3 = C[3+bs*0] + a_3 * b_3 * a_r;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
			D[3+bs*0] = c_3;
	

			a_r = Ar[1];
			
			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			b_2 = B[2+bs*1];
			b_3 = B[3+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*1] + a_1 * b_1 * a_r;
			c_2 = C[2+bs*1] + a_2 * b_2 * a_r;
			c_3 = C[3+bs*1] + a_3 * b_3 * a_r;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
			D[3+bs*1] = c_3;
	

			a_r = Ar[2];
			
			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			b_2 = B[2+bs*2];
			b_3 = B[3+bs*2];
			
			c_0 = C[0+bs*2] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*2] + a_1 * b_1 * a_r;
			c_2 = C[2+bs*2] + a_2 * b_2 * a_r;
			c_3 = C[3+bs*2] + a_3 * b_3 * a_r;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			D[2+bs*2] = c_2;
			D[3+bs*2] = c_3;
	

			a_r = Ar[3];
			
			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			b_2 = B[2+bs*3];
			b_3 = B[3+bs*3];
			
			c_0 = C[0+bs*3] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*3] + a_1 * b_1 * a_r;
			c_2 = C[2+bs*3] + a_2 * b_2 * a_r;
			c_3 = C[3+bs*3] + a_3 * b_3 * a_r;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
			D[2+bs*3] = c_2;
			D[3+bs*3] = c_3;
	
			Ar += 4;
			B  += 16;
			C  += 16;
			D  += 16;
			
			}

		a_r = Ar[0];

		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		b_2 = B[2+bs*0];
		b_3 = B[3+bs*0];
		
		c_0 = C[0+bs*0] + a_0 * b_0 * a_r;
		c_1 = C[1+bs*0] + a_1 * b_1 * a_r;
		c_2 = C[2+bs*0] + a_2 * b_2 * a_r;
		c_3 = C[3+bs*0] + a_3 * b_3 * a_r;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;
		D[2+bs*0] = c_2;
		D[3+bs*0] = c_3;


		a_r = Ar[1];
		
		b_1 = B[1+bs*1];
		b_2 = B[2+bs*1];
		b_3 = B[3+bs*1];
		
		c_1 = C[1+bs*1] + a_1 * b_1 * a_r;
		c_2 = C[2+bs*1] + a_2 * b_2 * a_r;
		c_3 = C[3+bs*1] + a_3 * b_3 * a_r;

		D[1+bs*1] = c_1;
		D[2+bs*1] = c_2;
		D[3+bs*1] = c_3;


		a_r = Ar[2];
		
		b_2 = B[2+bs*2];
		b_3 = B[3+bs*2];
		
		c_2 = C[2+bs*2] + a_2 * b_2 * a_r;
		c_3 = C[3+bs*2] + a_3 * b_3 * a_r;

		D[2+bs*2] = c_2;
		D[3+bs*2] = c_3;


		a_r = Ar[3];
		
		b_3 = B[3+bs*3];
		
		c_3 = C[3+bs*3] + a_3 * b_3 * a_r;

		D[3+bs*3] = c_3;

		}
	
	}



// Al and Ar are the diagonal of two matrices
void kernel_dsyrk_diag_left_right_3_lib4(int kmax, double *Al, double *Ar, double *B, double *C, double *D, int alg)
	{

	if(kmax<=0)
		return;
	
	// assume kmax to be multiple of 4
	
	const int bs = 4;

	int k;

	double
		a_r,
		a_0, a_1, a_2,
		b_0, b_1, b_2,
		c_0, c_1, c_2;
		
	if(alg==-1)
		{
		a_0 = - Al[0];
		a_1 = - Al[1];
		a_2 = - Al[2];
		}
	else
		{
		a_0 = Al[0];
		a_1 = Al[1];
		a_2 = Al[2];
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-3; k+=4)
			{

			a_r = Ar[0];
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			b_2 = B[2+bs*0];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;
			c_2 = a_2 * b_2 * a_r;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
	

			a_r = Ar[1];
			
			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			b_2 = B[2+bs*1];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;
			c_2 = a_2 * b_2 * a_r;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
	

			a_r = Ar[2];
			
			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			b_2 = B[2+bs*2];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;
			c_2 = a_2 * b_2 * a_r;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			D[2+bs*2] = c_2;
	

			a_r = Ar[3];
			
			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			b_2 = B[2+bs*3];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;
			c_2 = a_2 * b_2 * a_r;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
			D[2+bs*3] = c_2;
	
			Ar += 4;
			B  += 16;
			C  += 16;
			D  += 16;
			
			}

		a_r = Ar[0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		b_2 = B[2+bs*0];
		
		c_0 = a_0 * b_0 * a_r;
		c_1 = a_1 * b_1 * a_r;
		c_2 = a_2 * b_2 * a_r;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;
		D[2+bs*0] = c_2;


		a_r = Ar[1];
		
		b_1 = B[1+bs*1];
		b_2 = B[2+bs*1];
		
		c_1 = a_1 * b_1 * a_r;
		c_2 = a_2 * b_2 * a_r;

		D[1+bs*1] = c_1;
		D[2+bs*1] = c_2;


		a_r = Ar[2];
		
		b_2 = B[2+bs*2];
		
		c_2 = a_2 * b_2 * a_r;

		D[2+bs*2] = c_2;

		}
	else
		{

		for(k=0; k<kmax-3; k+=4)
			{

			a_r = Ar[0];
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			b_2 = B[2+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*0] + a_1 * b_1 * a_r;
			c_2 = C[2+bs*0] + a_2 * b_2 * a_r;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
			D[2+bs*0] = c_2;
	

			a_r = Ar[1];
			
			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			b_2 = B[2+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*1] + a_1 * b_1 * a_r;
			c_2 = C[2+bs*1] + a_2 * b_2 * a_r;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
			D[2+bs*1] = c_2;
	

			a_r = Ar[2];
			
			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			b_2 = B[2+bs*2];
			
			c_0 = C[0+bs*2] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*2] + a_1 * b_1 * a_r;
			c_2 = C[2+bs*2] + a_2 * b_2 * a_r;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
			D[2+bs*2] = c_2;
	

			a_r = Ar[3];
			
			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			b_2 = B[2+bs*3];
			
			c_0 = C[0+bs*3] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*3] + a_1 * b_1 * a_r;
			c_2 = C[2+bs*3] + a_2 * b_2 * a_r;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
			D[2+bs*3] = c_2;
	
			Ar += 4;
			B  += 16;
			C  += 16;
			D  += 16;
			
			}

		a_r = Ar[0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		b_2 = B[2+bs*0];
		
		c_0 = C[0+bs*0] + a_0 * b_0 * a_r;
		c_1 = C[1+bs*0] + a_1 * b_1 * a_r;
		c_2 = C[2+bs*0] + a_2 * b_2 * a_r;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;
		D[2+bs*0] = c_2;


		a_r = Ar[1];
		
		b_1 = B[1+bs*1];
		b_2 = B[2+bs*1];
		
		c_1 = C[1+bs*1] + a_1 * b_1 * a_r;
		c_2 = C[2+bs*1] + a_2 * b_2 * a_r;

		D[1+bs*1] = c_1;
		D[2+bs*1] = c_2;


		a_r = Ar[2];
		
		b_2 = B[2+bs*2];
		
		c_2 = C[2+bs*2] + a_2 * b_2 * a_r;

		D[2+bs*2] = c_2;

		}
	
	}


// Al and Ar are the diagonal of two matrices
void kernel_dsyrk_diag_left_right_2_lib4(int kmax, double *Al, double *Ar, double *B, double *C, double *D, int alg)
	{

	if(kmax<=0)
		return;
	
	// assume kmax to be multiple of 4
	
	const int bs = 4;

	int k;

	double
		a_r,
		a_0, a_1,
		b_0, b_1,
		c_0, c_1;
		
	if(alg==-1)
		{
		a_0 = - Al[0];
		a_1 = - Al[1];
		}
	else
		{
		a_0 = Al[0];
		a_1 = Al[1];
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-2; k+=4)
			{

			a_r = Ar[0];
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
	

			a_r = Ar[1];
			
			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
	

			a_r = Ar[2];
			
			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
	

			a_r = Ar[3];
			
			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			
			c_0 = a_0 * b_0 * a_r;
			c_1 = a_1 * b_1 * a_r;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
	
			Ar += 4;
			B  += 16;
			C  += 16;
			D  += 16;
			
			}

		a_r = Ar[0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		
		c_0 = a_0 * b_0 * a_r;
		c_1 = a_1 * b_1 * a_r;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;


		a_r = Ar[1];
		
		b_1 = B[1+bs*1];
		
		c_1 = a_1 * b_1 * a_r;

		D[1+bs*1] = c_1;

		}
	else
		{

		for(k=0; k<kmax-2; k+=4)
			{

			a_r = Ar[0];
			
			b_0 = B[0+bs*0];
			b_1 = B[1+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*0] + a_1 * b_1 * a_r;

			D[0+bs*0] = c_0;
			D[1+bs*0] = c_1;
	

			a_r = Ar[1];
			
			b_0 = B[0+bs*1];
			b_1 = B[1+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*1] + a_1 * b_1 * a_r;

			D[0+bs*1] = c_0;
			D[1+bs*1] = c_1;
	

			a_r = Ar[2];
			
			b_0 = B[0+bs*2];
			b_1 = B[1+bs*2];
			
			c_0 = C[0+bs*2] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*2] + a_1 * b_1 * a_r;

			D[0+bs*2] = c_0;
			D[1+bs*2] = c_1;
	

			a_r = Ar[3];
			
			b_0 = B[0+bs*3];
			b_1 = B[1+bs*3];
			
			c_0 = C[0+bs*3] + a_0 * b_0 * a_r;
			c_1 = C[1+bs*3] + a_1 * b_1 * a_r;

			D[0+bs*3] = c_0;
			D[1+bs*3] = c_1;
	
			Ar += 4;
			B  += 16;
			C  += 16;
			D  += 16;
			
			}

		a_r = Ar[0];
		
		b_0 = B[0+bs*0];
		b_1 = B[1+bs*0];
		
		c_0 = C[0+bs*0] + a_0 * b_0 * a_r;
		c_1 = C[1+bs*0] + a_1 * b_1 * a_r;

		D[0+bs*0] = c_0;
		D[1+bs*0] = c_1;


		a_r = Ar[1];
		
		b_1 = B[1+bs*1];
		
		c_1 = C[1+bs*1] + a_1 * b_1 * a_r;

		D[1+bs*1] = c_1;


		}
	
	}


// Al and Ar are the diagonal of two matrices
void kernel_dsyrk_diag_left_right_1_lib4(int kmax, double *Al, double *Ar, double *B, double *C, double *D, int alg)
	{

	if(kmax<=0)
		return;
	
	// assume kmax to be multiple of 4
	
	const int bs = 4;

	int k;

	double
		a_r,
		a_0,
		b_0,
		c_0;
		
	if(alg==-1)
		{
		a_0 = - Al[0];
		}
	else
		{
		a_0 = Al[0];
		}
	
	if(alg==0)
		{
		
		for(k=0; k<kmax-1; k+=4)
			{

			a_r = Ar[0];
			
			b_0 = B[0+bs*0];
			
			c_0 = a_0 * b_0 * a_r;

			D[0+bs*0] = c_0;
	

			a_r = Ar[1];
			
			b_0 = B[0+bs*1];
			
			c_0 = a_0 * b_0 * a_r;

			D[0+bs*1] = c_0;
	

			a_r = Ar[2];
			
			b_0 = B[0+bs*2];
			
			c_0 = a_0 * b_0 * a_r;

			D[0+bs*2] = c_0;
	

			a_r = Ar[3];
			
			b_0 = B[0+bs*3];
			
			c_0 = a_0 * b_0 * a_r;

			D[0+bs*3] = c_0;
	
			Ar += 4;
			B  += 16;
			C  += 16;
			D  += 16;
			
			}

		a_r = Ar[0];
		
		b_0 = B[0+bs*0];
		
		c_0 = a_0 * b_0 * a_r;

		D[0+bs*0] = c_0;

		}
	else
		{

		for(k=0; k<kmax-2; k+=4)
			{

			a_r = Ar[0];
			
			b_0 = B[0+bs*0];
			
			c_0 = C[0+bs*0] + a_0 * b_0 * a_r;

			D[0+bs*0] = c_0;
	

			a_r = Ar[1];
			
			b_0 = B[0+bs*1];
			
			c_0 = C[0+bs*1] + a_0 * b_0 * a_r;

			D[0+bs*1] = c_0;
	

			a_r = Ar[2];
			
			b_0 = B[0+bs*2];
			
			c_0 = C[0+bs*2] + a_0 * b_0 * a_r;

			D[0+bs*2] = c_0;
	

			a_r = Ar[3];
			
			b_0 = B[0+bs*3];
			
			c_0 = C[0+bs*3] + a_0 * b_0 * a_r;

			D[0+bs*3] = c_0;
	
			Ar += 4;
			B  += 16;
			C  += 16;
			D  += 16;
			
			}

		a_r = Ar[0];
		
		b_0 = B[0+bs*0];
		
		c_0 = C[0+bs*0] + a_0 * b_0 * a_r;

		D[0+bs*0] = c_0;

		}
	
	}




