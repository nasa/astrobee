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

#include <mmintrin.h>
#include <xmmintrin.h>  // SSE
/*#include <emmintrin.h>  // SSE2*/
/*#include <pmmintrin.h>  // SSE3*/
//#include <smmintrin.h>  // SSE4
//#include <immintrin.h>  // AVX
#include <math.h>

#include "../../include/block_size.h"



// normal-transposed, 8x4 with data packed in 4
void kernel_ssyrk_spotrf_nt_8x4_lib4(int kadd, int ksub, float *A0, float *A1, float *B, float *C0, float *C1, float *D0, float *D1, float *fact)
	{
	
	int ki_add = kadd/4;
	int kl_add = kadd%4;
	int ki_sub = ksub/4;

	const int bs = S_MR;//4;
	const int d_ncl = S_NCL;//2;

	long long dA = bs*((d_ncl-kadd%d_ncl)%d_ncl)*sizeof(float);

	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"movq          %3, %%rax         \n\t" // load address of A0
		"movq          %4, %%rcx         \n\t" // load address of A1
		"movq          %5, %%rbx         \n\t" // load address of B
		"                                \n\t"
		"                                \n\t"
		"movaps        0(%%rax), %%xmm0  \n\t" // initialize loop by pre-loading elements
		"movaps        0(%%rcx), %%xmm1  \n\t" // of a and b.
		"movaps        0(%%rbx), %%xmm2  \n\t"
		"                                \n\t"
		"                                \n\t"
		"xorpd     %%xmm3,  %%xmm3       \n\t"
		"movaps    %%xmm3,  %%xmm4       \n\t"
		"movaps    %%xmm3,  %%xmm5       \n\t"
		"movaps    %%xmm3,  %%xmm6       \n\t"
		"movaps    %%xmm3,  %%xmm7       \n\t"
		"movaps    %%xmm3,  %%xmm8       \n\t"
		"movaps    %%xmm3,  %%xmm9       \n\t"
		"movaps    %%xmm3, %%xmm10       \n\t"
		"movaps    %%xmm3, %%xmm11       \n\t"
		"movaps    %%xmm3, %%xmm12       \n\t"
		"movaps    %%xmm3, %%xmm13       \n\t"
		"movaps    %%xmm3, %%xmm14       \n\t"
		"movaps    %%xmm3, %%xmm15       \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movl      %0, %%esi             \n\t" // i = k_iter;
		"testl  %%esi, %%esi             \n\t" // check i via logical AND.
		"je     .CONSIDERADD_8X4        \n\t" // if i == 0, jump to code that
		"                                \n\t" // contains the k_left loop.
		"                                \n\t"
		"                                \n\t"
		".LOOPADD_8X4:                  \n\t" // MAIN LOOP add
		"                                \n\t"
		"addps   %%xmm6, %%xmm10         \n\t" // iteration 0
		"addps   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x39, %%xmm2, %%xmm7  \n\t"
		"mulps   %%xmm0, %%xmm2          \n\t"
		"mulps   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addps   %%xmm4, %%xmm11         \n\t"
		"addps   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"pshufd   $0x39, %%xmm7, %%xmm6  \n\t"
		"mulps   %%xmm0, %%xmm7          \n\t"
		"mulps   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addps   %%xmm2, %%xmm8          \n\t"
		"movaps       16(%%rbx), %%xmm2  \n\t"
		"addps   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x39, %%xmm6, %%xmm4  \n\t"
		"mulps   %%xmm0, %%xmm6          \n\t"
		"mulps   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addps   %%xmm7, %%xmm9          \n\t"
		"addps   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulps   %%xmm0, %%xmm4          \n\t"
		"movaps       16(%%rax), %%xmm0  \n\t"
		"mulps   %%xmm1, %%xmm5          \n\t"
		"movaps       16(%%rcx), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"addps   %%xmm6, %%xmm10         \n\t" // iteration 1
		"addps   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x39, %%xmm2, %%xmm7  \n\t"
		"mulps   %%xmm0, %%xmm2          \n\t"
		"mulps   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addps   %%xmm4, %%xmm11         \n\t"
		"addps   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"pshufd   $0x39, %%xmm7, %%xmm6  \n\t"
		"mulps   %%xmm0, %%xmm7          \n\t"
		"mulps   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addps   %%xmm2, %%xmm8          \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
		"addps   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x39, %%xmm6, %%xmm4  \n\t"
		"mulps   %%xmm0, %%xmm6          \n\t"
		"mulps   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addps   %%xmm7, %%xmm9          \n\t"
		"addps   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulps   %%xmm0, %%xmm4          \n\t"
		"movaps       32(%%rax), %%xmm0  \n\t"
		"mulps   %%xmm1, %%xmm5          \n\t"
		"movaps       32(%%rcx), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"addps   %%xmm6, %%xmm10         \n\t" // iteration 2
		"addps   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x39, %%xmm2, %%xmm7  \n\t"
		"mulps   %%xmm0, %%xmm2          \n\t"
		"mulps   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addps   %%xmm4, %%xmm11         \n\t"
		"addps   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"pshufd   $0x39, %%xmm7, %%xmm6  \n\t"
		"mulps   %%xmm0, %%xmm7          \n\t"
		"mulps   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addps   %%xmm2, %%xmm8          \n\t"
		"movaps       48(%%rbx), %%xmm2  \n\t"
		"addps   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x39, %%xmm6, %%xmm4  \n\t"
		"mulps   %%xmm0, %%xmm6          \n\t"
		"mulps   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addps   %%xmm7, %%xmm9          \n\t"
		"addps   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulps   %%xmm0, %%xmm4          \n\t"
		"movaps       48(%%rax), %%xmm0  \n\t"
		"mulps   %%xmm1, %%xmm5          \n\t"
		"movaps       48(%%rcx), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"addps   %%xmm6, %%xmm10         \n\t" // iteration 3
		"addps   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x39, %%xmm2, %%xmm7  \n\t"
		"mulps   %%xmm0, %%xmm2          \n\t"
		"mulps   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addq       $64, %%rax           \n\t" // A0 += 16
		"                                \n\t"
		"addps   %%xmm4, %%xmm11         \n\t"
		"addps   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"pshufd   $0x39, %%xmm7, %%xmm6  \n\t"
		"mulps   %%xmm0, %%xmm7          \n\t"
		"mulps   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addq       $64, %%rbx           \n\t" // B += 16
		"                                \n\t"
		"addps   %%xmm2, %%xmm8          \n\t"
		"movaps         (%%rbx), %%xmm2  \n\t"
		"addps   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x39, %%xmm6, %%xmm4  \n\t"
		"mulps   %%xmm0, %%xmm6          \n\t"
		"mulps   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addq       $64, %%rcx           \n\t" // A1 += 16
		"                                \n\t"
		"addps   %%xmm7, %%xmm9          \n\t"
		"decl    %%esi                   \n\t" // i -= 1;
		"addps   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulps   %%xmm0, %%xmm4          \n\t"
		"movaps         (%%rax), %%xmm0  \n\t"
		"mulps   %%xmm1, %%xmm5          \n\t"
		"movaps         (%%rcx), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"jne    .LOOPADD_8X4            \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".CONSIDERADD_8X4:              \n\t"
		"                                \n\t"
		"movl      %1, %%esi             \n\t" // i = k_left;
		"testl  %%esi, %%esi             \n\t" // check i via logical AND.
		"je     .CONSIDERSUB_8X4        \n\t" // if i == 0, we're done; jump to end.
		"                                \n\t" // else, we prepare to enter k_left loop.
		"                                \n\t"
		"                                \n\t"
		".LOOPLEFT_8X4:                 \n\t" // EDGE LOOP
		"                                \n\t"
		"addps   %%xmm6, %%xmm10         \n\t" // iteration 0
		"addps   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x39, %%xmm2, %%xmm7  \n\t"
		"mulps   %%xmm0, %%xmm2          \n\t"
		"mulps   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addps   %%xmm4, %%xmm11         \n\t"
		"addps   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"pshufd   $0x39, %%xmm7, %%xmm6  \n\t"
		"mulps   %%xmm0, %%xmm7          \n\t"
		"mulps   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addps   %%xmm2, %%xmm8          \n\t"
		"movaps       16(%%rbx), %%xmm2  \n\t"
		"addps   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x39, %%xmm6, %%xmm4  \n\t"
		"mulps   %%xmm0, %%xmm6          \n\t"
		"mulps   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addps   %%xmm7, %%xmm9          \n\t"
		"addps   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulps   %%xmm0, %%xmm4          \n\t"
		"movaps       16(%%rax), %%xmm0  \n\t"
		"mulps   %%xmm1, %%xmm5          \n\t"
		"movaps       16(%%rcx), %%xmm1  \n\t"
		"                                \n\t"
		"addq          $16, %%rax        \n\t" // A0 += 4
		"addq          $16, %%rbx        \n\t" // B  += 4
		"addq          $16, %%rcx        \n\t" // A1 += 4
		"                                \n\t"
		"                                \n\t"
		"decl    %%esi                   \n\t" // i -= 1;
		"jne    .LOOPLEFT_8X4           \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".CONSIDERSUB_8X4:              \n\t" // padd
		"                                \n\t"
		"                                \n\t"
		"addps   %%xmm6, %%xmm10         \n\t"
		"addps   %%xmm3, %%xmm14         \n\t"
		"addps   %%xmm4, %%xmm11         \n\t"
		"addps   %%xmm5, %%xmm15         \n\t"
		"                                \n\t"
		"xorpd     %%xmm3,  %%xmm3       \n\t"
		"movaps    %%xmm3,  %%xmm4       \n\t"
		"movaps    %%xmm3,  %%xmm5       \n\t"
		"movaps    %%xmm3,  %%xmm6       \n\t"
		"                                \n\t"
		"                                \n\t"
		"movl   %2, %%edx                \n\t"
		"cmpl	$0, %%edx                \n\t"
		"jle    .POSTACC_8X4            \n\t"
		"                                \n\t"
		"movq   %11, %%rdx                \n\t"
		"cmpq	$0, %%rdx                \n\t"
		"jle    .PRELOOPSUB_8X4         \n\t"
		"                                \n\t"
		"movl   %0, %%edx                \n\t"
		"movl   %1, %%edx                \n\t"
		"addl   %%edx, %%edx              \n\t"
		"cmpl	$0, %%edx                \n\t"
		"jle    .PRELOOPSUB_8X4         \n\t"
		"                                \n\t"
		"movq   %11, %%rdx                \n\t"
		"addq   %%rdx, %%rax             \n\t" 
		"addq   %%rdx, %%rbx             \n\t" 
		"addq   %%rdx, %%rcx             \n\t" 
		"                                \n\t"
		"movaps        0(%%rax), %%xmm0  \n\t" // initialize loop by pre-loading elements
		"movaps        0(%%rcx), %%xmm1  \n\t" // of a and b.
		"movaps        0(%%rbx), %%xmm2  \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".PRELOOPSUB_8X4:               \n\t" // 
		"                                \n\t"
		"movl   %2, %%esi                \n\t"
		"                                \n\t"
		"                                \n\t"
		".LOOPSUB_8X4:                  \n\t" // main loop 2
		"                                \n\t"
		"subps   %%xmm6, %%xmm10         \n\t" // iteration 0
		"subps   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x39, %%xmm2, %%xmm7  \n\t"
		"mulps   %%xmm0, %%xmm2          \n\t"
		"mulps   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subps   %%xmm4, %%xmm11         \n\t"
		"subps   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"pshufd   $0x39, %%xmm7, %%xmm6  \n\t"
		"mulps   %%xmm0, %%xmm7          \n\t"
		"mulps   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subps   %%xmm2, %%xmm8          \n\t"
		"movaps       16(%%rbx), %%xmm2  \n\t"
		"subps   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x39, %%xmm6, %%xmm4  \n\t"
		"mulps   %%xmm0, %%xmm6          \n\t"
		"mulps   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subps   %%xmm7, %%xmm9          \n\t"
		"subps   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulps   %%xmm0, %%xmm4          \n\t"
		"movaps       16(%%rax), %%xmm0  \n\t"
		"mulps   %%xmm1, %%xmm5          \n\t"
		"movaps       16(%%rcx), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"subps   %%xmm6, %%xmm10         \n\t" // iteration 1
		"subps   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x39, %%xmm2, %%xmm7  \n\t"
		"mulps   %%xmm0, %%xmm2          \n\t"
		"mulps   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subps   %%xmm4, %%xmm11         \n\t"
		"subps   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"pshufd   $0x39, %%xmm7, %%xmm6  \n\t"
		"mulps   %%xmm0, %%xmm7          \n\t"
		"mulps   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subps   %%xmm2, %%xmm8          \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
		"subps   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x39, %%xmm6, %%xmm4  \n\t"
		"mulps   %%xmm0, %%xmm6          \n\t"
		"mulps   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subps   %%xmm7, %%xmm9          \n\t"
		"subps   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulps   %%xmm0, %%xmm4          \n\t"
		"movaps       32(%%rax), %%xmm0  \n\t"
		"mulps   %%xmm1, %%xmm5          \n\t"
		"movaps       32(%%rcx), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"subps   %%xmm6, %%xmm10         \n\t" // iteration 2
		"subps   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x39, %%xmm2, %%xmm7  \n\t"
		"mulps   %%xmm0, %%xmm2          \n\t"
		"mulps   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subps   %%xmm4, %%xmm11         \n\t"
		"subps   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"pshufd   $0x39, %%xmm7, %%xmm6  \n\t"
		"mulps   %%xmm0, %%xmm7          \n\t"
		"mulps   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subps   %%xmm2, %%xmm8          \n\t"
		"movaps       48(%%rbx), %%xmm2  \n\t"
		"subps   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x39, %%xmm6, %%xmm4  \n\t"
		"mulps   %%xmm0, %%xmm6          \n\t"
		"mulps   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subps   %%xmm7, %%xmm9          \n\t"
		"subps   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulps   %%xmm0, %%xmm4          \n\t"
		"movaps       48(%%rax), %%xmm0  \n\t"
		"mulps   %%xmm1, %%xmm5          \n\t"
		"movaps       48(%%rcx), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"subps   %%xmm6, %%xmm10         \n\t" // iteration 3
		"subps   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x39, %%xmm2, %%xmm7  \n\t"
		"mulps   %%xmm0, %%xmm2          \n\t"
		"mulps   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addq       $64, %%rax           \n\t" // A0 += 16
		"                                \n\t"
		"subps   %%xmm4, %%xmm11         \n\t"
		"subps   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"pshufd   $0x39, %%xmm7, %%xmm6  \n\t"
		"mulps   %%xmm0, %%xmm7          \n\t"
		"mulps   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addq       $64, %%rbx           \n\t" // B += 16
		"                                \n\t"
		"subps   %%xmm2, %%xmm8          \n\t"
		"movaps         (%%rbx), %%xmm2  \n\t"
		"subps   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x39, %%xmm6, %%xmm4  \n\t"
		"mulps   %%xmm0, %%xmm6          \n\t"
		"mulps   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addq       $64, %%rcx           \n\t" // A1 += 16
		"                                \n\t"
		"subps   %%xmm7, %%xmm9          \n\t"
		"decl    %%esi                   \n\t" // i -= 1;
		"subps   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
		"mulps   %%xmm0, %%xmm4          \n\t"
		"movaps         (%%rax), %%xmm0  \n\t"
		"mulps   %%xmm1, %%xmm5          \n\t"
		"movaps         (%%rcx), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"jne    .LOOPSUB_8X4             \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"subps   %%xmm6, %%xmm10         \n\t"
		"subps   %%xmm3, %%xmm14         \n\t"
		"subps   %%xmm4, %%xmm11         \n\t"
		"subps   %%xmm5, %%xmm15         \n\t"
		"                                \n\t"
		"                                \n\t"
		".POSTACC_8X4:                   \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movaps  %%xmm9, %%xmm4          \n\t"
		"shufps   $0xd8, %%xmm8,  %%xmm9 \n\t"
		"shufps   $0xd8, %%xmm11, %%xmm8 \n\t"
		"shufps   $0xd8, %%xmm10, %%xmm11\n\t"
		"shufps   $0xd8, %%xmm4,  %%xmm10\n\t"
		"                                \n\t"
		"movaps  %%xmm8, %%xmm4          \n\t"
		"shufps   $0xd8, %%xmm10, %%xmm8 \n\t"
		"shufps   $0xd8, %%xmm4, %%xmm10 \n\t"
		"movaps  %%xmm9, %%xmm5          \n\t"
		"shufps   $0xd8, %%xmm11, %%xmm9 \n\t"
		"shufps   $0xd8, %%xmm5, %%xmm11 \n\t"
		"                                \n\t"
		"movaps  %%xmm13, %%xmm4         \n\t"
		"shufps   $0xd8, %%xmm12, %%xmm13\n\t"
		"shufps   $0xd8, %%xmm15, %%xmm12\n\t"
		"shufps   $0xd8, %%xmm14, %%xmm15\n\t"
		"shufps   $0xd8, %%xmm4,  %%xmm14\n\t"
		"                                \n\t"
		"movaps  %%xmm12, %%xmm4         \n\t"
		"shufps   $0xd8, %%xmm14, %%xmm12\n\t"
		"shufps   $0xd8, %%xmm4, %%xmm14 \n\t"
		"movaps  %%xmm13, %%xmm5         \n\t"
		"shufps   $0xd8, %%xmm15, %%xmm13\n\t"
		"shufps   $0xd8, %%xmm5, %%xmm15 \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movq   %6, %%rax                \n\t" // load address of C0
		"movq   %7, %%rbx                \n\t" // load address of C1
		"                                \n\t"
		"                                \n\t"
		"movaps  (%%rax),   %%xmm0       \n\t" // load C0
		"movaps  16(%%rax), %%xmm1       \n\t"
		"movaps  32(%%rax), %%xmm2       \n\t"
		"movaps  48(%%rax), %%xmm3       \n\t"
		"movaps  (%%rbx),   %%xmm4       \n\t" // load C0
		"movaps  16(%%rbx), %%xmm5       \n\t"
		"movaps  32(%%rbx), %%xmm6       \n\t"
		"movaps  48(%%rbx), %%xmm7       \n\t"
		"                                \n\t"
		"                                \n\t"
		"addps  %%xmm0, %%xmm8           \n\t"
		"addps  %%xmm1, %%xmm9           \n\t"
		"addps  %%xmm2, %%xmm10          \n\t"
		"addps  %%xmm3, %%xmm11          \n\t"
		"addps  %%xmm4, %%xmm12          \n\t"
		"addps  %%xmm5, %%xmm13          \n\t"
		"addps  %%xmm6, %%xmm14          \n\t"
		"addps  %%xmm7, %%xmm15          \n\t"
		"                                \n\t"
		"                                \n\t" //  8  9 10 11
		"                                \n\t" // 12 13 14 15
		"                                \n\t"
		"movq   %8, %%rax                \n\t" // load address of D0
		"movq   %9, %%rbx                \n\t" // load address of D1
		"movq   %10, %%rcx                \n\t" // load address of fact
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movss  .THR_8X4(%%rip), %%xmm7  \n\t" // 1e-7
		"movss  .ONE_8X4(%%rip), %%xmm6  \n\t" // 1.0
		"                                \n\t"
		"                                \n\t"
		".A00_8X4:                       \n\t"
		"movss   %%xmm8, %%xmm0          \n\t" // a_00
		"ucomiss %%xmm7, %%xmm0          \n\t"
		"jbe     .ZERO0_8X4              \n\t"
		"sqrtss  %%xmm0, %%xmm0          \n\t"
		"movss   %%xmm6, %%xmm5          \n\t" // 1.0
		"divss   %%xmm0, %%xmm5          \n\t" // 1.0/a_00
		"movss   %%xmm5, (%%rcx)         \n\t" // fact[0]
		"shufps  $0x0, %%xmm5, %%xmm5    \n\t"
		"mulps   %%xmm5, %%xmm8          \n\t"
		"mulps   %%xmm5, %%xmm12         \n\t"
		"movaps	 %%xmm8,  (%%rax)        \n\t"
		"movaps	 %%xmm12, (%%rbx)        \n\t"
		"                                \n\t"
		".A11_8X4:                       \n\t"
		"movaps  %%xmm8, %%xmm0          \n\t"
		"shufps  $0x55, %%xmm0, %%xmm0   \n\t" // a_10
		"movss   %%xmm0, 4(%%rcx)        \n\t" // fact[1]
		"movaps  %%xmm0,  %%xmm1         \n\t"
		"mulps   %%xmm8, %%xmm0          \n\t"
		"mulps   %%xmm12, %%xmm1         \n\t"
		"subps   %%xmm0, %%xmm9          \n\t"
		"subps   %%xmm1, %%xmm13         \n\t"
		"movaps  %%xmm9, %%xmm0          \n\t" // [a_01 a_11]
		"shufps  $0x55, %%xmm0, %%xmm0   \n\t" // a_11
		"ucomiss %%xmm7, %%xmm0          \n\t"
		"jbe     .ZERO1_8X4              \n\t"
		"sqrtss  %%xmm0, %%xmm0          \n\t"
		"movss   %%xmm6, %%xmm5          \n\t" // 1.0
		"divss   %%xmm0, %%xmm5          \n\t" // 1.0/a_11
		"movss   16(%%rax), %%xmm0       \n\t"
		"movss   %%xmm5, 8(%%rcx)        \n\t" // fact[2]
		"shufps  $0x0, %%xmm5, %%xmm5    \n\t"
		"mulps   %%xmm5, %%xmm9          \n\t"
		"mulps   %%xmm5, %%xmm13         \n\t"
		"movss   %%xmm0, %%xmm9          \n\t"
		"movaps	 %%xmm9,  16(%%rax)      \n\t"
		"movaps	 %%xmm13, 16(%%rbx)      \n\t"
		"                                \n\t"
		".A22_8X4:                       \n\t"
		"movaps  %%xmm8, %%xmm0          \n\t" // a_20
		"movaps  %%xmm9, %%xmm1          \n\t" // a_21
		"shufps  $0xaa, %%xmm0, %%xmm0   \n\t" // a_20
		"shufps  $0xaa, %%xmm1, %%xmm1   \n\t" // a_21
		"movss   %%xmm0, 12(%%rcx)       \n\t" // fact[3]
		"movss   %%xmm1, 16(%%rcx)       \n\t" // fact[4]
		"movaps  %%xmm0, %%xmm2          \n\t" // a_20
		"movaps  %%xmm1, %%xmm3          \n\t" // a_20
		"mulps   %%xmm8, %%xmm0          \n\t"
		"mulps   %%xmm12, %%xmm2          \n\t"
		"mulps   %%xmm9, %%xmm1          \n\t"
		"mulps   %%xmm13, %%xmm3          \n\t"
		"subps   %%xmm0, %%xmm10         \n\t"
		"subps   %%xmm2, %%xmm14         \n\t"
		"subps   %%xmm1, %%xmm10         \n\t"
		"subps   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm10, %%xmm0         \n\t"
		"shufps  $0xaa, %%xmm0, %%xmm0   \n\t" // a_22
		"ucomiss %%xmm7, %%xmm0          \n\t"
		"jbe     .ZERO2_8X4              \n\t"
		"sqrtss  %%xmm0, %%xmm0          \n\t"
		"movss   %%xmm6, %%xmm5          \n\t" // 1.0
		"divss   %%xmm0, %%xmm5          \n\t" // 1.0/a_11
		"movaps	 32(%%rax), %%xmm0       \n\t"
		"movss   %%xmm5, 20(%%rcx)       \n\t" // fact[5]
		"shufps  $0x00, %%xmm5, %%xmm5   \n\t"
		"mulps   %%xmm5, %%xmm10         \n\t"
		"mulps   %%xmm5, %%xmm14         \n\t"
		"shufps  $0xe4, %%xmm10, %%xmm0  \n\t"
		"movaps	 %%xmm0, %%xmm10         \n\t"
		"movaps	 %%xmm0, 32(%%rax)       \n\t"
		"movaps	 %%xmm14, 32(%%rbx)      \n\t"
		"                                \n\t"
		".A33_8X4:                       \n\t"
		"movaps  %%xmm8, %%xmm0          \n\t"
		"movaps  %%xmm9, %%xmm1          \n\t"
		"movaps  %%xmm10, %%xmm2         \n\t"
		"shufps  $0xff, %%xmm0, %%xmm0   \n\t" // a_30
		"shufps  $0xff, %%xmm1, %%xmm1   \n\t" // a_31
		"shufps  $0xff, %%xmm2, %%xmm2   \n\t" // a_32
		"movss   %%xmm0, 24(%%rcx)       \n\t" // fact[6]
		"movss   %%xmm1, 28(%%rcx)       \n\t" // fact[7]
		"movss   %%xmm2, 32(%%rcx)       \n\t" // fact[8]
		"movaps  %%xmm0, %%xmm3          \n\t"
		"movaps  %%xmm1, %%xmm4          \n\t"
		"movaps  %%xmm2, %%xmm5          \n\t"
		"mulps   %%xmm8, %%xmm0          \n\t"
		"mulps   %%xmm12, %%xmm3         \n\t"
		"mulps   %%xmm9, %%xmm1          \n\t"
		"mulps   %%xmm13, %%xmm4         \n\t"
		"subps   %%xmm0, %%xmm11         \n\t"
		"subps   %%xmm3, %%xmm15         \n\t"
		"mulps   %%xmm10, %%xmm2         \n\t"
		"mulps   %%xmm14, %%xmm5         \n\t"
		"subps   %%xmm1, %%xmm11         \n\t"
		"subps   %%xmm4, %%xmm15         \n\t"
		"subps   %%xmm2, %%xmm11         \n\t"
		"subps   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm11, %%xmm0         \n\t" // [a_23 a_33]
		"shufps  $0xff, %%xmm0, %%xmm0   \n\t" // a_33
		"ucomiss %%xmm7, %%xmm0          \n\t"
		"jbe     .ZERO3_8X4              \n\t"
		"sqrtss  %%xmm0, %%xmm0          \n\t"
		"movss   %%xmm0, 60(%%rax)       \n\t"
		"divss   %%xmm0, %%xmm6          \n\t" // 1.0/a_11
		"movss   %%xmm6, 36(%%rcx)       \n\t" // fact[9]
		"shufps  $0x00, %%xmm6, %%xmm6   \n\t" // a_33
		"mulps   %%xmm6, %%xmm15         \n\t"
		"movaps  %%xmm15, 48(%%rbx)      \n\t"
		"                                \n\t"
		"jmp    .DONE_8X4                \n\t"
		"                                \n\t"
		"                                \n\t"
		".ZERO0_8X4:                     \n\t"
		"xorpd  %%xmm8,  %%xmm8          \n\t"
		"movaps %%xmm8,  %%xmm12         \n\t"
		"movaps %%xmm8,  (%%rax)         \n\t"
		"movaps %%xmm12, (%%rbx)         \n\t"
		"movss  %%xmm8,  (%%rcx)         \n\t" // fact[0]
		"jmp    .A11_8X4                 \n\t"
		"                                \n\t"
		".ZERO1_8X4:                     \n\t"
		"xorpd  %%xmm9,  %%xmm9          \n\t"
		"movaps %%xmm9,  %%xmm13         \n\t"
		"movss  16(%%rax), %%xmm0        \n\t"
		"movss  %%xmm0,  %%xmm9          \n\t"
		"movaps %%xmm9,  16(%%rax)       \n\t"
		"movaps %%xmm13, 16(%%rbx)       \n\t"
		"movss  %%xmm9,  8(%%rcx)        \n\t" // fact[2]
		"jmp    .A22_8X4                 \n\t"
		"                                \n\t"
		".ZERO2_8X4:                     \n\t"
		"xorpd  %%xmm10, %%xmm10         \n\t"
		"movaps	 32(%%rax), %%xmm0       \n\t"
		"movaps %%xmm10, %%xmm14         \n\t"
		"shufps  $0xe4, %%xmm10, %%xmm0  \n\t"
		"movaps	 %%xmm0, %%xmm10         \n\t"
		"movaps %%xmm10, 32(%%rax)       \n\t"
		"movaps %%xmm14, 32(%%rbx)       \n\t"
		"movss  %%xmm10, 20(%%rcx)       \n\t" // fact[5]
		"jmp    .A33_8X4                 \n\t"
		"                                \n\t"
		".ZERO3_8X4:                     \n\t"
		"xorpd  %%xmm11, %%xmm11         \n\t"
		"movaps %%xmm11, %%xmm15         \n\t"
		"movss  %%xmm11, 60(%%rax)      \n\t"
		"movaps %%xmm15, 48(%%rbx)      \n\t"
		"movss  %%xmm11, 36(%%rcx)       \n\t" // fact[2]
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"jmp    .DONE_8X4                \n\t"
		".align 16                       \n\t"
		".ONE_8X4:                       \n\t"
		".long  1065353216               \n\t"
		".long  0                        \n\t"
		".long  0                        \n\t"
		".long  0                        \n\t"
		".align 16                       \n\t"
		".THR_8X4:                       \n\t"
		".long  869711765                \n\t"
		".long  0                        \n\t"
		".long  0                        \n\t"
		".long  0                        \n\t"
		"                                \n\t"
		"                                \n\t"
		".DONE_8X4:                      \n\t"
		"                                \n\t"
		: // output operands (none)
		: // input operands
		  "m" (ki_add),		// %0
		  "m" (kl_add),		// %1
		  "m" (ki_sub),		// %2
		  "m" (A0),			// %3
		  "m" (A1),			// %4
		  "m" (B),			// %5
		  "m" (C0),			// %6
		  "m" (C1),			// %7
		  "m" (D0),			// %8
		  "m" (D1),			// %9
		  "m" (fact),		// %10
		  "m" (dA)			// %11
		: // register clobber list
		  "rax", "rbx", "rcx", "rdx", "rsi", //"rdx", //"rdi", "r8", "r9", "r10", "r11",
		  "xmm0", "xmm1", "xmm2", "xmm3",
		  "xmm4", "xmm5", "xmm6", "xmm7",
		  "xmm8", "xmm9", "xmm10", "xmm11",
		  "xmm12", "xmm13", "xmm14", "xmm15",
		  "memory"
	);
}



void kernel_ssyrk_spotrf_nt_4x4_lib4(int kadd, int ksub, float *A0, float *B, float *C0, float *D0, float *fact)
	{
	
	int ki_add = kadd/4;
	int kl_add = kadd%4;
	int ki_sub = ksub/4;

	const int bs = S_MR;//4;
	const int d_ncl = S_NCL;//2;

	long long dA = bs*((d_ncl-kadd%d_ncl)%d_ncl)*sizeof(float);

	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"movq          %3, %%rax         \n\t" // load address of A0
		"movq          %4, %%rbx         \n\t" // load address of B
		"                                \n\t"
		"                                \n\t"
		"movaps        0(%%rax), %%xmm0  \n\t" // initialize loop by pre-loading elements
		"movaps       16(%%rax), %%xmm1  \n\t"
		"movaps        0(%%rbx), %%xmm2  \n\t"
		"                                \n\t"
		"                                \n\t"
		"xorpd     %%xmm3,  %%xmm3       \n\t"
		"movaps    %%xmm3,  %%xmm4       \n\t"
		"movaps    %%xmm3,  %%xmm5       \n\t"
		"movaps    %%xmm3,  %%xmm6       \n\t"
		"movaps    %%xmm3,  %%xmm7       \n\t"
		"movaps    %%xmm3,  %%xmm8       \n\t"
		"movaps    %%xmm3,  %%xmm9       \n\t"
		"movaps    %%xmm3, %%xmm10       \n\t"
		"movaps    %%xmm3, %%xmm11       \n\t"
		"movaps    %%xmm3, %%xmm12       \n\t"
		"movaps    %%xmm3, %%xmm13       \n\t"
		"movaps    %%xmm3, %%xmm14       \n\t"
		"movaps    %%xmm3, %%xmm15       \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movl      %0, %%esi             \n\t" // i = k_iter;
		"testl  %%esi, %%esi             \n\t" // check i via logical AND.
		"je     .CONSIDERADD_4X4        \n\t" // if i == 0, jump to code that
		"                                \n\t" // contains the k_left loop.
		"                                \n\t"
		"                                \n\t"
		".LOOPADD_4X4:                  \n\t" // MAIN LOOP add
		"                                \n\t"
		"addps   %%xmm6, %%xmm10         \n\t" // iteration 0
		"pshufd   $0x39, %%xmm2, %%xmm7  \n\t"
		"mulps   %%xmm0, %%xmm2          \n\t"
		"                                \n\t"
		"addps   %%xmm4, %%xmm11         \n\t"
		"pshufd   $0x39, %%xmm7, %%xmm6  \n\t"
		"mulps   %%xmm0, %%xmm7          \n\t"
		"                                \n\t"
		"addps   %%xmm2, %%xmm8          \n\t"
		"movaps       16(%%rbx), %%xmm2  \n\t"
		"pshufd   $0x39, %%xmm6, %%xmm4  \n\t"
		"mulps   %%xmm0, %%xmm6          \n\t"
		"                                \n\t"
		"addps   %%xmm7, %%xmm9          \n\t"
		"mulps   %%xmm0, %%xmm4          \n\t"
		"movaps       32(%%rax), %%xmm0  \n\t"
		"                                \n\t"
		"                                \n\t"
		"addps   %%xmm6, %%xmm10         \n\t" // iteration 1
		"pshufd   $0x39, %%xmm2, %%xmm7  \n\t"
		"mulps   %%xmm1, %%xmm2          \n\t"
		"                                \n\t"
		"addps   %%xmm4, %%xmm11         \n\t"
		"pshufd   $0x39, %%xmm7, %%xmm6  \n\t"
		"mulps   %%xmm1, %%xmm7          \n\t"
		"                                \n\t"
		"addps   %%xmm2, %%xmm8          \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
		"pshufd   $0x39, %%xmm6, %%xmm4  \n\t"
		"mulps   %%xmm1, %%xmm6          \n\t"
		"                                \n\t"
		"addps   %%xmm7, %%xmm9          \n\t"
		"mulps   %%xmm1, %%xmm4          \n\t"
		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"addps   %%xmm6, %%xmm10         \n\t" // iteration 2
		"pshufd   $0x39, %%xmm2, %%xmm7  \n\t"
		"mulps   %%xmm0, %%xmm2          \n\t"
		"                                \n\t"
		"addps   %%xmm4, %%xmm11         \n\t"
		"pshufd   $0x39, %%xmm7, %%xmm6  \n\t"
		"mulps   %%xmm0, %%xmm7          \n\t"
		"                                \n\t"
		"addps   %%xmm2, %%xmm8          \n\t"
		"movaps       48(%%rbx), %%xmm2  \n\t"
		"pshufd   $0x39, %%xmm6, %%xmm4  \n\t"
		"mulps   %%xmm0, %%xmm6          \n\t"
		"                                \n\t"
		"addps   %%xmm7, %%xmm9          \n\t"
		"mulps   %%xmm0, %%xmm4          \n\t"
		"movaps       64(%%rax), %%xmm0  \n\t"
		"                                \n\t"
		"                                \n\t"
		"addps   %%xmm6, %%xmm10         \n\t" // iteration 3
		"pshufd   $0x39, %%xmm2, %%xmm7  \n\t"
		"mulps   %%xmm1, %%xmm2          \n\t"
		"addq       $64, %%rax           \n\t" // A0 += 16
		"                                \n\t"
		"addps   %%xmm4, %%xmm11         \n\t"
		"pshufd   $0x39, %%xmm7, %%xmm6  \n\t"
		"mulps   %%xmm1, %%xmm7          \n\t"
		"addq       $64, %%rbx           \n\t" // B += 16
		"                                \n\t"
		"addps   %%xmm2, %%xmm8          \n\t"
		"movaps         (%%rbx), %%xmm2  \n\t"
		"pshufd   $0x39, %%xmm6, %%xmm4  \n\t"
		"mulps   %%xmm1, %%xmm6          \n\t"
		"                                \n\t"
		"addps   %%xmm7, %%xmm9          \n\t"
		"decl    %%esi                   \n\t" // i -= 1;
		"mulps   %%xmm1, %%xmm4          \n\t"
		"movaps       16(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"jne    .LOOPADD_4X4            \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".CONSIDERADD_4X4:              \n\t"
		"                                \n\t"
		"movl      %1, %%esi             \n\t" // i = k_left;
		"testl  %%esi, %%esi             \n\t" // check i via logical AND.
		"je     .CONSIDERSUB_4X4        \n\t" // if i == 0, we're done; jump to end.
		"                                \n\t" // else, we prepare to enter k_left loop.
		"                                \n\t"
		"                                \n\t"
		".LOOPLEFT_4X4:                 \n\t" // EDGE LOOP
		"                                \n\t"
		"addps   %%xmm6, %%xmm10         \n\t" // iteration 0
		"pshufd   $0x39, %%xmm2, %%xmm7  \n\t"
		"mulps   %%xmm0, %%xmm2          \n\t"
		"                                \n\t"
		"addps   %%xmm4, %%xmm11         \n\t"
		"pshufd   $0x39, %%xmm7, %%xmm6  \n\t"
		"mulps   %%xmm0, %%xmm7          \n\t"
		"                                \n\t"
		"addps   %%xmm2, %%xmm8          \n\t"
		"movaps       16(%%rbx), %%xmm2  \n\t"
		"pshufd   $0x39, %%xmm6, %%xmm4  \n\t"
		"mulps   %%xmm0, %%xmm6          \n\t"
		"                                \n\t"
		"addps   %%xmm7, %%xmm9          \n\t"
		"mulps   %%xmm0, %%xmm4          \n\t"
		"movaps       16(%%rax), %%xmm0  \n\t"
		"                                \n\t"
		"addq          $16, %%rax        \n\t" // A0 += 4
		"addq          $16, %%rbx        \n\t" // B += 4
		"                                \n\t"
		"                                \n\t"
		"decl    %%esi                   \n\t" // i -= 1;
		"jne    .LOOPLEFT_4X4           \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".CONSIDERSUB_4X4:              \n\t" // padd
		"                                \n\t"
		"                                \n\t"
		"addps   %%xmm6, %%xmm10         \n\t"
		"addps   %%xmm4, %%xmm11         \n\t"
		"                                \n\t"
		"xorpd     %%xmm4,  %%xmm4       \n\t"
		"movaps    %%xmm4,  %%xmm6       \n\t"
		"                                \n\t"
		"                                \n\t"
		"movl   %2, %%edx                \n\t"
		"cmpl	$0, %%edx                \n\t"
		"jle    .POSTACC_4X4            \n\t"
		"                                \n\t"
		"movq   %8, %%rdx                \n\t"
		"cmpq	$0, %%rdx                \n\t"
		"jle    .PRELOOPSUB_4X4         \n\t"
		"                                \n\t"
		"movl   %0, %%edx                \n\t"
		"movl   %1, %%edx                \n\t"
		"addl   %%edx, %%edx              \n\t"
		"cmpl	$0, %%edx                \n\t"
		"jle    .PRELOOPSUB_4X4         \n\t"
		"                                \n\t"
		"movq   %8, %%rdx                \n\t"
		"addq   %%rdx, %%rax             \n\t" 
		"addq   %%rdx, %%rbx             \n\t" 
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".PRELOOPSUB_4X4:               \n\t" // 
		"                                \n\t"
		"movaps        0(%%rax), %%xmm0  \n\t" // initialize loop by pre-loading elements
		"movaps       16(%%rax), %%xmm1  \n\t"
		"movaps        0(%%rbx), %%xmm2  \n\t"
		"                                \n\t"
		"movl   %2, %%esi                \n\t"
		"                                \n\t"
		"                                \n\t"
		".LOOPSUB_4X4:                  \n\t" // main loop 2
		"                                \n\t"
		"subps   %%xmm6, %%xmm10         \n\t" // iteration 0
		"pshufd   $0x39, %%xmm2, %%xmm7  \n\t"
		"mulps   %%xmm0, %%xmm2          \n\t"
		"                                \n\t"
		"subps   %%xmm4, %%xmm11         \n\t"
		"pshufd   $0x39, %%xmm7, %%xmm6  \n\t"
		"mulps   %%xmm0, %%xmm7          \n\t"
		"                                \n\t"
		"subps   %%xmm2, %%xmm8          \n\t"
		"movaps       16(%%rbx), %%xmm2  \n\t"
		"pshufd   $0x39, %%xmm6, %%xmm4  \n\t"
		"mulps   %%xmm0, %%xmm6          \n\t"
		"                                \n\t"
		"subps   %%xmm7, %%xmm9          \n\t"
		"mulps   %%xmm0, %%xmm4          \n\t"
		"movaps       32(%%rax), %%xmm0  \n\t"
		"                                \n\t"
		"                                \n\t"
		"subps   %%xmm6, %%xmm10         \n\t" // iteration 1
		"pshufd   $0x39, %%xmm2, %%xmm7  \n\t"
		"mulps   %%xmm1, %%xmm2          \n\t"
		"                                \n\t"
		"subps   %%xmm4, %%xmm11         \n\t"
		"pshufd   $0x39, %%xmm7, %%xmm6  \n\t"
		"mulps   %%xmm1, %%xmm7          \n\t"
		"                                \n\t"
		"subps   %%xmm2, %%xmm8          \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
		"pshufd   $0x39, %%xmm6, %%xmm4  \n\t"
		"mulps   %%xmm1, %%xmm6          \n\t"
		"                                \n\t"
		"subps   %%xmm7, %%xmm9          \n\t"
		"mulps   %%xmm1, %%xmm4          \n\t"
		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"subps   %%xmm6, %%xmm10         \n\t" // iteration 2
		"pshufd   $0x39, %%xmm2, %%xmm7  \n\t"
		"mulps   %%xmm0, %%xmm2          \n\t"
		"                                \n\t"
		"subps   %%xmm4, %%xmm11         \n\t"
		"pshufd   $0x39, %%xmm7, %%xmm6  \n\t"
		"mulps   %%xmm0, %%xmm7          \n\t"
		"                                \n\t"
		"subps   %%xmm2, %%xmm8          \n\t"
		"movaps       48(%%rbx), %%xmm2  \n\t"
		"pshufd   $0x39, %%xmm6, %%xmm4  \n\t"
		"mulps   %%xmm0, %%xmm6          \n\t"
		"                                \n\t"
		"subps   %%xmm7, %%xmm9          \n\t"
		"mulps   %%xmm0, %%xmm4          \n\t"
		"movaps       64(%%rax), %%xmm0  \n\t"
		"                                \n\t"
		"                                \n\t"
		"subps   %%xmm6, %%xmm10         \n\t" // iteration 3
		"pshufd   $0x39, %%xmm2, %%xmm7  \n\t"
		"mulps   %%xmm1, %%xmm2          \n\t"
		"addq       $64, %%rax           \n\t" // A0 += 16
		"                                \n\t"
		"subps   %%xmm4, %%xmm11         \n\t"
		"pshufd   $0x39, %%xmm7, %%xmm6  \n\t"
		"mulps   %%xmm1, %%xmm7          \n\t"
		"addq       $64, %%rbx           \n\t" // B += 16
		"                                \n\t"
		"subps   %%xmm2, %%xmm8          \n\t"
		"movaps         (%%rbx), %%xmm2  \n\t"
		"pshufd   $0x39, %%xmm6, %%xmm4  \n\t"
		"mulps   %%xmm1, %%xmm6          \n\t"
		"                                \n\t"
		"subps   %%xmm7, %%xmm9          \n\t"
		"decl    %%esi                   \n\t" // i -= 1;
		"mulps   %%xmm1, %%xmm4          \n\t"
		"movaps       16(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"jne    .LOOPSUB_4X4             \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"subps   %%xmm6, %%xmm10         \n\t"
		"subps   %%xmm4, %%xmm11         \n\t"
		"                                \n\t"
		"                                \n\t"
		".POSTACC_4X4:                   \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movaps  %%xmm9, %%xmm4          \n\t"
		"shufps   $0xd8, %%xmm8,  %%xmm9 \n\t"
		"shufps   $0xd8, %%xmm11, %%xmm8 \n\t"
		"shufps   $0xd8, %%xmm10, %%xmm11\n\t"
		"shufps   $0xd8, %%xmm4,  %%xmm10\n\t"
		"                                \n\t"
		"movaps  %%xmm8, %%xmm4          \n\t"
		"shufps   $0xd8, %%xmm10, %%xmm8 \n\t"
		"shufps   $0xd8, %%xmm4, %%xmm10 \n\t"
		"movaps  %%xmm9, %%xmm5          \n\t"
		"shufps   $0xd8, %%xmm11, %%xmm9 \n\t"
		"shufps   $0xd8, %%xmm5, %%xmm11 \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movq   %5, %%rax                \n\t" // load address of C0
		"                                \n\t"
		"                                \n\t"
		"movaps  (%%rax),   %%xmm0       \n\t" // load C0
		"movaps  16(%%rax), %%xmm1       \n\t"
		"movaps  32(%%rax), %%xmm2       \n\t"
		"movaps  48(%%rax), %%xmm3       \n\t"
		"                                \n\t"
		"                                \n\t"
		"addps  %%xmm0, %%xmm8           \n\t"
		"addps  %%xmm1, %%xmm9           \n\t"
		"addps  %%xmm2, %%xmm10          \n\t"
		"addps  %%xmm3, %%xmm11          \n\t"
		"                                \n\t"
		"                                \n\t" //  8  9 10 11
		"                                \n\t" // 12 13 14 15
		"                                \n\t"
		"movq   %6, %%rax                \n\t" // load address of D0
		"movq   %7, %%rcx                \n\t" // load address of fact
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movss  .THR_4X4(%%rip), %%xmm7  \n\t" // 1e-7
		"movss  .ONE_4X4(%%rip), %%xmm6  \n\t" // 1.0
		"                                \n\t"
		"                                \n\t"
		".A00_4X4:                       \n\t"
		"movss   %%xmm8, %%xmm0          \n\t" // a_00
		"ucomiss %%xmm7, %%xmm0          \n\t"
		"jbe     .ZERO0_4X4              \n\t"
		"sqrtss  %%xmm0, %%xmm0          \n\t"
		"movss   %%xmm6, %%xmm5          \n\t" // 1.0
		"divss   %%xmm0, %%xmm5          \n\t" // 1.0/a_00
		"movss   %%xmm5, (%%rcx)         \n\t" // fact[0]
		"shufps  $0x0, %%xmm5, %%xmm5    \n\t"
		"mulps   %%xmm5, %%xmm8          \n\t"
		"movaps	 %%xmm8,  (%%rax)        \n\t"
		"                                \n\t"
		".A11_4X4:                       \n\t"
		"movaps  %%xmm8, %%xmm0          \n\t"
		"shufps  $0x55, %%xmm0, %%xmm0   \n\t" // a_10
		"movss   %%xmm0, 4(%%rcx)        \n\t" // fact[1]
		"mulps   %%xmm8, %%xmm0          \n\t"
		"subps   %%xmm0, %%xmm9          \n\t"
		"movaps  %%xmm9, %%xmm0          \n\t" // [a_01 a_11]
		"shufps  $0x55, %%xmm0, %%xmm0   \n\t" // a_11
		"ucomiss %%xmm7, %%xmm0          \n\t"
		"jbe     .ZERO1_4X4              \n\t"
		"sqrtss  %%xmm0, %%xmm0          \n\t"
		"movss   %%xmm6, %%xmm5          \n\t" // 1.0
		"divss   %%xmm0, %%xmm5          \n\t" // 1.0/a_11
		"movss   16(%%rax), %%xmm0       \n\t"
		"movss   %%xmm5, 8(%%rcx)        \n\t" // fact[2]
		"shufps  $0x0, %%xmm5, %%xmm5    \n\t"
		"mulps   %%xmm5, %%xmm9          \n\t"
		"movss   %%xmm0, %%xmm9          \n\t"
		"movaps	 %%xmm9,  16(%%rax)      \n\t"
		"                                \n\t"
		".A22_4X4:                       \n\t"
		"movaps  %%xmm8, %%xmm0          \n\t" // a_20
		"movaps  %%xmm9, %%xmm1          \n\t" // a_21
		"shufps  $0xaa, %%xmm0, %%xmm0   \n\t" // a_20
		"shufps  $0xaa, %%xmm1, %%xmm1   \n\t" // a_21
		"movss   %%xmm0, 12(%%rcx)       \n\t" // fact[3]
		"movss   %%xmm1, 16(%%rcx)       \n\t" // fact[4]
		"mulps   %%xmm8, %%xmm0          \n\t"
		"mulps   %%xmm9, %%xmm1          \n\t"
		"subps   %%xmm0, %%xmm10         \n\t"
		"subps   %%xmm1, %%xmm10         \n\t"
		"movaps  %%xmm10, %%xmm0         \n\t"
		"shufps  $0xaa, %%xmm0, %%xmm0   \n\t" // a_22
		"ucomiss %%xmm7, %%xmm0          \n\t"
		"jbe     .ZERO2_4X4              \n\t"
		"sqrtss  %%xmm0, %%xmm0          \n\t"
		"movss   %%xmm6, %%xmm5          \n\t" // 1.0
		"divss   %%xmm0, %%xmm5          \n\t" // 1.0/a_11
		"movaps	 32(%%rax), %%xmm0       \n\t"
		"movss   %%xmm5, 20(%%rcx)       \n\t" // fact[5]
		"shufps  $0x00, %%xmm5, %%xmm5   \n\t"
		"mulps   %%xmm5, %%xmm10         \n\t"
		"shufps  $0xe4, %%xmm10, %%xmm0  \n\t"
		"movaps	 %%xmm0, %%xmm10         \n\t"
		"movaps	 %%xmm0, 32(%%rax)       \n\t"
		"                                \n\t"
		".A33_4X4:                       \n\t"
		"movaps  %%xmm8, %%xmm0          \n\t"
		"movaps  %%xmm9, %%xmm1          \n\t"
		"movaps  %%xmm10, %%xmm2         \n\t"
		"shufps  $0xff, %%xmm0, %%xmm0   \n\t" // a_30
		"shufps  $0xff, %%xmm1, %%xmm1   \n\t" // a_31
		"shufps  $0xff, %%xmm2, %%xmm2   \n\t" // a_32
		"movss   %%xmm0, 24(%%rcx)       \n\t" // fact[6]
		"movss   %%xmm1, 28(%%rcx)       \n\t" // fact[7]
		"movss   %%xmm2, 32(%%rcx)       \n\t" // fact[8]
		"mulps   %%xmm8, %%xmm0          \n\t"
		"mulps   %%xmm9, %%xmm1          \n\t"
		"subps   %%xmm0, %%xmm11         \n\t"
		"mulps   %%xmm10, %%xmm2         \n\t"
		"subps   %%xmm1, %%xmm11         \n\t"
		"subps   %%xmm2, %%xmm11         \n\t"
		"movaps  %%xmm11, %%xmm0         \n\t" // [a_23 a_33]
		"shufps  $0xff, %%xmm0, %%xmm0   \n\t" // a_33
		"ucomiss %%xmm7, %%xmm0          \n\t"
		"jbe     .ZERO3_4X4              \n\t"
		"sqrtss  %%xmm0, %%xmm0          \n\t"
		"movss   %%xmm0, 60(%%rax)       \n\t"
		"divss   %%xmm0, %%xmm6          \n\t" // 1.0/a_11
		"movss   %%xmm6, 36(%%rcx)       \n\t" // fact[9]
		"                                \n\t"
		"jmp    .DONE_4X4                \n\t"
		"                                \n\t"
		"                                \n\t"
		".ZERO0_4X4:                     \n\t"
		"xorpd  %%xmm8,  %%xmm8          \n\t"
		"movaps %%xmm8,  (%%rax)         \n\t"
		"movss  %%xmm8,  (%%rcx)         \n\t" // fact[0]
		"jmp    .A11_4X4                 \n\t"
		"                                \n\t"
		".ZERO1_4X4:                     \n\t"
		"xorpd  %%xmm9,  %%xmm9          \n\t"
		"movss  16(%%rax), %%xmm0        \n\t"
		"movss  %%xmm0,  %%xmm9          \n\t"
		"movaps %%xmm9,  16(%%rax)       \n\t"
		"movss  %%xmm9,  8(%%rcx)        \n\t" // fact[2]
		"jmp    .A22_4X4                 \n\t"
		"                                \n\t"
		".ZERO2_4X4:                     \n\t"
		"xorpd  %%xmm10, %%xmm10         \n\t"
		"movaps	 32(%%rax), %%xmm0       \n\t"
		"shufps  $0xe4, %%xmm10, %%xmm0  \n\t"
		"movaps	 %%xmm0, %%xmm10         \n\t"
		"movaps %%xmm10, 32(%%rax)       \n\t"
		"movss  %%xmm10, 20(%%rcx)       \n\t" // fact[5]
		"jmp    .A33_4X4                 \n\t"
		"                                \n\t"
		".ZERO3_4X4:                     \n\t"
		"xorpd  %%xmm11, %%xmm11         \n\t"
		"movss  %%xmm11, 60(%%rax)      \n\t"
		"movss  %%xmm11, 36(%%rcx)       \n\t" // fact[2]
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"jmp    .DONE_4X4                \n\t"
		".align 16                       \n\t"
		".ONE_4X4:                       \n\t"
		".long  1065353216               \n\t"
		".long  0                        \n\t"
		".long  0                        \n\t"
		".long  0                        \n\t"
		".align 16                       \n\t"
		".THR_4X4:                       \n\t"
		".long  869711765                \n\t"
		".long  0                        \n\t"
		".long  0                        \n\t"
		".long  0                        \n\t"
		"                                \n\t"
		"                                \n\t"
		".DONE_4X4:                      \n\t"
		"                                \n\t"
		: // output operands (none)
		: // input operands
		  "m" (ki_add),		// %0
		  "m" (kl_add),		// %1
		  "m" (ki_sub),		// %2
		  "m" (A0),			// %3
		  "m" (B),			// %4
		  "m" (C0),			// %5
		  "m" (D0),			// %6
		  "m" (fact),		// %7
		  "m" (dA)			// %8
		: // register clobber list
		  "rax", "rbx", "rcx", "rdx", "rsi", //"rdx", //"rdi", "r8", "r9", "r10", "r11",
		  "xmm0", "xmm1", "xmm2", "xmm3",
		  "xmm4", "xmm5", "xmm6", "xmm7",
		  "xmm8", "xmm9", "xmm10", "xmm11",
		  "xmm12", "xmm13", "xmm14", "xmm15",
		  "memory"
	);
}



void kernel_ssyrk_spotrf_nt_4x4_lib4_old(int kadd, int ksub, float *A, float *B, float *C, float *D, float *fact)
	{

	const int bs = 4;
	const int d_ncl = S_NCL;
	const int lda = bs;
	const int ldc = bs;

	int k;

	float
		a_0, a_1, a_2, a_3,
		b_0, b_1, b_2, b_3,
		c_00=0, 
		c_10=0, c_11=0, 
		c_20=0, c_21=0, c_22=0, 
		c_30=0, c_31=0, c_32=0, c_33=0;
		
	for(k=0; k<kadd-3; k+=4)
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

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

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

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

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

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

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

		c_11 += a_1 * b_1;
		c_21 += a_2 * b_1;
		c_31 += a_3 * b_1;

		c_22 += a_2 * b_2;
		c_32 += a_3 * b_2;

		c_33 += a_3 * b_3;
		
		
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
		b_2 = B[2+lda*0];
		b_3 = B[3+lda*0];
		
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

/*	for(k=0; k<ksub-3; k+=4)*/
/*		{*/
/*		*/
/*		a_0 = A[0+lda*0];*/
/*		a_1 = A[1+lda*0];*/
/*		a_2 = A[2+lda*0];*/
/*		a_3 = A[3+lda*0];*/
/*		*/
/*		b_0 = B[0+lda*0];*/
/*		b_1 = B[1+lda*0];*/
/*		b_2 = B[2+lda*0];*/
/*		b_3 = B[3+lda*0];*/
/*		*/
/*		c_00 -= a_0 * b_0;*/
/*		c_10 -= a_1 * b_0;*/
/*		c_20 -= a_2 * b_0;*/
/*		c_30 -= a_3 * b_0;*/

/*		c_11 -= a_1 * b_1;*/
/*		c_21 -= a_2 * b_1;*/
/*		c_31 -= a_3 * b_1;*/

/*		c_22 -= a_2 * b_2;*/
/*		c_32 -= a_3 * b_2;*/

/*		c_33 -= a_3 * b_3;*/


/*		a_0 = A[0+lda*1];*/
/*		a_1 = A[1+lda*1];*/
/*		a_2 = A[2+lda*1];*/
/*		a_3 = A[3+lda*1];*/
/*		*/
/*		b_0 = B[0+lda*1];*/
/*		b_1 = B[1+lda*1];*/
/*		b_2 = B[2+lda*1];*/
/*		b_3 = B[3+lda*1];*/
/*		*/
/*		c_00 -= a_0 * b_0;*/
/*		c_10 -= a_1 * b_0;*/
/*		c_20 -= a_2 * b_0;*/
/*		c_30 -= a_3 * b_0;*/

/*		c_11 -= a_1 * b_1;*/
/*		c_21 -= a_2 * b_1;*/
/*		c_31 -= a_3 * b_1;*/

/*		c_22 -= a_2 * b_2;*/
/*		c_32 -= a_3 * b_2;*/

/*		c_33 -= a_3 * b_3;*/


/*		a_0 = A[0+lda*2];*/
/*		a_1 = A[1+lda*2];*/
/*		a_2 = A[2+lda*2];*/
/*		a_3 = A[3+lda*2];*/
/*		*/
/*		b_0 = B[0+lda*2];*/
/*		b_1 = B[1+lda*2];*/
/*		b_2 = B[2+lda*2];*/
/*		b_3 = B[3+lda*2];*/
/*		*/
/*		c_00 -= a_0 * b_0;*/
/*		c_10 -= a_1 * b_0;*/
/*		c_20 -= a_2 * b_0;*/
/*		c_30 -= a_3 * b_0;*/

/*		c_11 -= a_1 * b_1;*/
/*		c_21 -= a_2 * b_1;*/
/*		c_31 -= a_3 * b_1;*/

/*		c_22 -= a_2 * b_2;*/
/*		c_32 -= a_3 * b_2;*/

/*		c_33 -= a_3 * b_3;*/


/*		a_0 = A[0+lda*3];*/
/*		a_1 = A[1+lda*3];*/
/*		a_2 = A[2+lda*3];*/
/*		a_3 = A[3+lda*3];*/
/*		*/
/*		b_0 = B[0+lda*3];*/
/*		b_1 = B[1+lda*3];*/
/*		b_2 = B[2+lda*3];*/
/*		b_3 = B[3+lda*3];*/
/*		*/
/*		c_00 -= a_0 * b_0;*/
/*		c_10 -= a_1 * b_0;*/
/*		c_20 -= a_2 * b_0;*/
/*		c_30 -= a_3 * b_0;*/

/*		c_11 -= a_1 * b_1;*/
/*		c_21 -= a_2 * b_1;*/
/*		c_31 -= a_3 * b_1;*/

/*		c_22 -= a_2 * b_2;*/
/*		c_32 -= a_3 * b_2;*/

/*		c_33 -= a_3 * b_3;*/
/*		*/
/*		*/
/*		A += 16;*/
/*		B += 16;*/

/*		}*/




	D[0+ldc*0] = c_00;
	D[1+ldc*0] = c_10;
	D[2+ldc*0] = c_20;
	D[3+ldc*0] = c_30;

	D[1+ldc*1] = c_11;
	D[2+ldc*1] = c_21;
	D[3+ldc*1] = c_31;

	D[2+ldc*2] = c_22;
	D[3+ldc*2] = c_32;

	D[3+ldc*3] = c_33;
	
	return;



	c_00 += C[0+ldc*0];
	c_10 += C[1+ldc*0];
	c_20 += C[2+ldc*0];
	c_30 += C[3+ldc*0];

	c_11 += C[1+ldc*1];
	c_21 += C[2+ldc*1];
	c_31 += C[3+ldc*1];

	c_22 += C[2+ldc*2];
	c_32 += C[3+ldc*2];

	c_33 += C[3+ldc*3];
	
	// spotrf
	
	// first column
	if(c_00 > 1e-7)
		{
		c_00 = sqrt(c_00);
		D[0+ldc*0] = c_00;
		c_00 = 1.0/c_00;
		c_10 *= c_00;
		c_20 *= c_00;
		c_30 *= c_00;
		}
	else
		{
		c_00 = 0.0;
		D[0+ldc*0] = c_00;
		c_10 = 0.0;
		c_20 = 0.0;
		c_30 = 0.0;
		}
	D[1+ldc*0] = c_10;
	D[2+ldc*0] = c_20;
	D[3+ldc*0] = c_30;
	
	// second column
	c_11 -= c_10*c_10;
	if(c_11 > 1e-7)
		{
		c_11 = sqrt(c_11);
		D[1+ldc*1] = c_11;
		c_11 = 1.0/c_11;
		c_21 -= c_20*c_10;
		c_31 -= c_30*c_10;
		c_21 *= c_11;
		c_31 *= c_11;
		}
	else
		{
		c_11 = 0.0;
		D[1+ldc*1] = c_11;
		c_21 = 0.0;
		c_31 = 0.0;
		}
	D[2+ldc*1] = c_21;
	D[3+ldc*1] = c_31;

	// third column
	c_22 -= c_20*c_20;
	c_22 -= c_21*c_21;
	if(c_22 > 1e-7)
		{
		c_22 = sqrt(c_22);
		D[2+ldc*2] = c_22;
		c_22 = 1.0/c_22;
		c_32 -= c_30*c_20;
		c_32 -= c_31*c_21;
		c_32 *= c_22;
		}
	else
		{
		c_22 = 0.0;
		D[2+ldc*2] = c_22;
		c_32 = 0.0;
		}
	D[3+ldc*2] = c_32;

	// fourth column
	c_33 -= c_30*c_30;
	c_33 -= c_31*c_31;
	c_33 -= c_32*c_32;
	if(c_33 > 1e-7)
		{
		c_33 = sqrt(c_33);
		D[3+ldc*3] = c_33;
		c_33 = 1.0/c_33;
		}
	else
		{
		c_33 = 0.0;
		D[3+ldc*3] = c_33;
		}
	
	// save factorized matrix with reciprocal of diagonal
	fact[0] = c_00;
	fact[1] = c_10;
	fact[3] = c_20;
	fact[6] = c_30;
	fact[2] = c_11;
	fact[4] = c_21;
	fact[7] = c_31;
	fact[5] = c_22;
	fact[8] = c_32;
	fact[9] = c_33;

	}



void kernel_ssyrk_spotrf_nt_4x2_lib4(int kadd, int ksub, float *A, float *B, float *C, float *D, float *fact)
	{

	const int bs = 4;
	const int d_ncl = S_NCL;
	const int lda = bs;
	const int ldc = bs;

	int k;

	float
		a_0, a_1, a_2, a_3,
		b_0, b_1,
		c_00=0, 
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

	c_11 += C[1+ldc*1];
	c_21 += C[2+ldc*1];
	c_31 += C[3+ldc*1];
	
	// spotrf
	
	// first column
	if(c_00 > 1e-7)
		{
		c_00 = sqrt(c_00);
		D[0+ldc*0] = c_00;
		c_00 = 1.0/c_00;
		c_10 *= c_00;
		c_20 *= c_00;
		c_30 *= c_00;
		}
	else
		{
		c_00 = 0.0;
		D[0+ldc*0] = c_00;
		c_10 = 0.0;
		c_20 = 0.0;
		c_30 = 0.0;
		}
	D[1+ldc*0] = c_10;
	D[2+ldc*0] = c_20;
	D[3+ldc*0] = c_30;
	
	// second column
	c_11 -= c_10*c_10;
	if(c_11 > 1e-7)
		{
		c_11 = sqrt(c_11);
		D[1+ldc*1] = c_11;
		c_11 = 1.0/c_11;
		c_21 -= c_20*c_10;
		c_31 -= c_30*c_10;
		c_21 *= c_11;
		c_31 *= c_11;
		}
	else
		{
		c_11 = 0.0;
		D[1+ldc*1] = c_11;
		c_21 = 0.0;
		c_31 = 0.0;
		}
	D[2+ldc*1] = c_21;
	D[3+ldc*1] = c_31;

	// save factorized matrix with reciprocal of diagonal
	fact[0] = c_00;
	fact[1] = c_10;
	fact[3] = c_20;
	fact[6] = c_30;
	fact[2] = c_11;
	fact[4] = c_21;
	fact[7] = c_31;
/*	fact[5] = c_22;*/
/*	fact[8] = c_32;*/
/*	fact[9] = c_33;*/

	}



void kernel_ssyrk_spotrf_nt_2x2_lib4(int kadd, int ksub, float *A, float *B, float *C, float *D, float *fact)
	{

	const int bs = 4;
	const int d_ncl = S_NCL;
	const int lda = bs;
	const int ldc = bs;

	int k;

	float
		a_0, a_1,
		b_0, b_1,
		c_00=0, 
		c_10=0, c_11=0;
		
	for(k=0; k<kadd-3; k+=4)
		{
		
		a_0 = A[0+lda*0];
		a_1 = A[1+lda*0];
		
		b_0 = B[0+lda*0];
		b_1 = B[1+lda*0];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_11 += a_1 * b_1;


		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		
		b_0 = B[0+lda*1];
		b_1 = B[1+lda*1];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_11 += a_1 * b_1;


		a_0 = A[0+lda*2];
		a_1 = A[1+lda*2];
		
		b_0 = B[0+lda*2];
		b_1 = B[1+lda*2];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

		c_11 += a_1 * b_1;


		a_0 = A[0+lda*3];
		a_1 = A[1+lda*3];
		
		b_0 = B[0+lda*3];
		b_1 = B[1+lda*3];
		
		c_00 += a_0 * b_0;
		c_10 += a_1 * b_0;

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

		c_11 -= a_1 * b_1;


		a_0 = A[0+lda*1];
		a_1 = A[1+lda*1];
		
		b_0 = B[0+lda*1];
		b_1 = B[1+lda*1];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_11 -= a_1 * b_1;


		a_0 = A[0+lda*2];
		a_1 = A[1+lda*2];
		
		b_0 = B[0+lda*2];
		b_1 = B[1+lda*2];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_11 -= a_1 * b_1;


		a_0 = A[0+lda*3];
		a_1 = A[1+lda*3];
		
		b_0 = B[0+lda*3];
		b_1 = B[1+lda*3];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_11 -= a_1 * b_1;
		
		
		A += 16;
		B += 16;

		}

	c_00 += C[0+ldc*0];
	c_10 += C[1+ldc*0];

	c_11 += C[1+ldc*1];
	
	// spotrf
	
	// first column
	if(c_00 > 1e-7)
		{
		c_00 = sqrt(c_00);
		D[0+ldc*0] = c_00;
		c_00 = 1.0/c_00;
		c_10 *= c_00;
		}
	else
		{
		c_00 = 0.0;
		D[0+ldc*0] = c_00;
		c_10 = 0.0;
		}
	D[1+ldc*0] = c_10;
	
	// second column
	c_11 -= c_10*c_10;
	if(c_11 > 1e-7)
		{
		c_11 = sqrt(c_11);
		D[1+ldc*1] = c_11;
		}
	else
		{
		c_11 = 0.0;
		D[1+ldc*1] = c_11;
		}

	// save factorized matrix with reciprocal of diagonal
	fact[0] = c_00;
	fact[1] = c_10;
/*	fact[3] = c_20;*/
/*	fact[6] = c_30;*/
	fact[2] = c_11;
/*	fact[4] = c_21;*/
/*	fact[7] = c_31;*/
/*	fact[5] = c_22;*/
/*	fact[8] = c_32;*/
/*	fact[9] = c_33;*/

	}




