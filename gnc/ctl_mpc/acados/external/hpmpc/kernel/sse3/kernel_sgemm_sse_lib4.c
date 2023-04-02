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



// normal-transposed, 8x4 with data packed in 4
void kernel_sgemm_nt_8x4_lib4(int kmax, float *A0, float *A1, float *B, float *C0, float *C1, float *D0, float *D1, int alg)
	{
	
	if(kmax<=0)
		return;
	
	int k_iter = kmax / 4;
	int k_left = kmax % 4;

	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"movq          %2, %%rax         \n\t" // load address of A0
		"movq          %3, %%rcx         \n\t" // load address of A1
		"movq          %4, %%rbx         \n\t" // load address of B
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
		"je     .SCONSIDKLEFT_8X4        \n\t" // if i == 0, jump to code that
		"                                \n\t" // contains the k_left loop.
		"                                \n\t"
		"                                \n\t"
		".SLOOPKITER_8X4:                \n\t" // MAIN LOOP
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
		"jne    .SLOOPKITER_8X4          \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".SCONSIDKLEFT_8X4:              \n\t"
		"                                \n\t"
		"movl      %1, %%esi             \n\t" // i = k_left;
		"testl  %%esi, %%esi             \n\t" // check i via logical AND.
		"je     .SPOSTACCUM_8X4          \n\t" // if i == 0, we're done; jump to end.
		"                                \n\t" // else, we prepare to enter k_left loop.
		"                                \n\t"
		"                                \n\t"
		".SLOOPKLEFT_8X4:                \n\t" // EDGE LOOP
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
		"addq          $16, %%rcx        \n\t" // A1 += 4
		"addq          $16, %%rbx        \n\t" // B += 4
		"                                \n\t"
		"                                \n\t"
		"decl    %%esi                   \n\t" // i -= 1;
		"jne    .SLOOPKLEFT_8X4          \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".SPOSTACCUM_8X4:                \n\t"
		"                                \n\t"
		"addps   %%xmm6, %%xmm10         \n\t"
		"addps   %%xmm3, %%xmm14         \n\t"
		"addps   %%xmm4, %%xmm11         \n\t"
		"addps   %%xmm5, %%xmm15         \n\t"
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
		"movq   %5, %%rax                \n\t" // load address of C0
		"movq   %6, %%rbx                \n\t" // load address of C1
		"movq   %8, %%r8                 \n\t" // load address of D0
		"movq   %9, %%r9                 \n\t" // load address of D1
		"                                \n\t"
		"                                \n\t"
		"movl   %7, %%ecx                \n\t" // alg
		"testl  %%ecx, %%ecx             \n\t" // check alg
		"je     .S0_8X4                  \n\t" // if alg==0, jump
		"                                \n\t"
		"cmpl	$1, %%ecx                \n\t"
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
		"je     .S1_8X4                  \n\t" // if alg==1, jump
		"                                \n\t"
		"                                \n\t"// alg==-1
		"subps  %%xmm8,  %%xmm0           \n\t"
		"subps  %%xmm9,  %%xmm1           \n\t"
		"subps  %%xmm10, %%xmm2           \n\t"
		"subps  %%xmm11, %%xmm3           \n\t"
		"subps  %%xmm12, %%xmm4           \n\t"
		"subps  %%xmm13, %%xmm5           \n\t"
		"subps  %%xmm14, %%xmm6           \n\t"
		"subps  %%xmm15, %%xmm7           \n\t"
		"                                \n\t"
		"movaps  %%xmm0, (%%r8)         \n\t"
		"movaps  %%xmm1, 16(%%r8)       \n\t"
		"movaps  %%xmm2, 32(%%r8)       \n\t"
		"movaps  %%xmm3, 48(%%r8)       \n\t"
		"movaps  %%xmm4, (%%r9)         \n\t"
		"movaps  %%xmm5, 16(%%r9)       \n\t"
		"movaps  %%xmm6, 32(%%r9)       \n\t"
		"movaps  %%xmm7, 48(%%r9)       \n\t"
		"                                \n\t"
		"jmp    .SDONE_8X4               \n\t" // jump to end
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".S1_8X4:                        \n\t" // alg==1
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
		".S0_8X4:                        \n\t" // alg==0
		"                                \n\t"
		"movaps	%%xmm8,  (%%r8)          \n\t"
		"movaps	%%xmm9,  16(%%r8)        \n\t"
		"movaps	%%xmm10, 32(%%r8)        \n\t"
		"movaps	%%xmm11, 48(%%r8)        \n\t"
		"movaps	%%xmm12, (%%r9)          \n\t"
		"movaps	%%xmm13, 16(%%r9)        \n\t"
		"movaps	%%xmm14, 32(%%r9)        \n\t"
		"movaps	%%xmm15, 48(%%r9)        \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".SDONE_8X4:                     \n\t"
		"                                \n\t"

		: // output operands (none)
		: // input operands
		  "m" (k_iter),		// %0
		  "m" (k_left),		// %1
		  "m" (A0),			// %2
		  "m" (A1),			// %3
		  "m" (B),			// %4
		  "m" (C0),			// %5
		  "m" (C1),			// %6
		  "m" (alg),		// %7
		  "m" (D0),			// %8
		  "m" (D1)			// %9
		: // register clobber list
		  "rax", "rbx", "rcx", "r8", "r9", "rsi", //"rdx", "rdi", "r8", "r9", "r10", "r11",
		  "xmm0", "xmm1", "xmm2", "xmm3",
		  "xmm4", "xmm5", "xmm6", "xmm7",
		  "xmm8", "xmm9", "xmm10", "xmm11",
		  "xmm12", "xmm13", "xmm14", "xmm15",
		  "memory"
	);
}



// normal-transposed, 4x4 with data packed in 4
void kernel_sgemm_nt_4x4_lib4(int kmax, float *A0, float *B, float *C0, float *D0, int alg)
	{
	
	if(kmax<=0)
		return;
	
	int k_iter = kmax / 4;
	int k_left = kmax % 4;

	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"movq          %2, %%rax         \n\t" // load address of A0
		"movq          %3, %%rbx         \n\t" // load address of B
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
/*		"movaps    %%xmm3, %%xmm12       \n\t"*/
/*		"movaps    %%xmm3, %%xmm13       \n\t"*/
/*		"movaps    %%xmm3, %%xmm14       \n\t"*/
/*		"movaps    %%xmm3, %%xmm15       \n\t"*/
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movl      %0, %%esi             \n\t" // i = k_iter;
		"testl  %%esi, %%esi             \n\t" // check i via logical AND.
		"je     .SCONSIDKLEFT_4X4        \n\t" // if i == 0, jump to code that
		"                                \n\t" // contains the k_left loop.
		"                                \n\t"
		"                                \n\t"
		".SLOOPKITER_4X4:                \n\t" // MAIN LOOP
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
		"jne    .SLOOPKITER_4X4          \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".SCONSIDKLEFT_4X4:              \n\t"
		"                                \n\t"
		"movl      %1, %%esi             \n\t" // i = k_left;
		"testl  %%esi, %%esi             \n\t" // check i via logical AND.
		"je     .SPOSTACCUM_4X4          \n\t" // if i == 0, we're done; jump to end.
		"                                \n\t" // else, we prepare to enter k_left loop.
		"                                \n\t"
		"                                \n\t"
		".SLOOPKLEFT_4X4:                \n\t" // EDGE LOOP
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
		"jne    .SLOOPKLEFT_4X4          \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".SPOSTACCUM_4X4:                \n\t"
		"                                \n\t"
		"addps   %%xmm6, %%xmm10         \n\t"
		"addps   %%xmm4, %%xmm11         \n\t"
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
		"movq   %4, %%rax                \n\t" // load address of C0
		"movq   %6, %%r8                 \n\t" // load address of D0
		"                                \n\t"
		"                                \n\t"
		"movl   %5, %%ecx                \n\t" // alg
		"testl  %%ecx, %%ecx             \n\t" // check alg
		"je     .S0_4X4                  \n\t" // if alg==0, jump
		"                                \n\t"
		"cmpl	$1, %%ecx                \n\t"
		"                                \n\t"
		"movaps  (%%rax),   %%xmm0       \n\t" // load C0
		"movaps  16(%%rax), %%xmm1       \n\t"
		"movaps  32(%%rax), %%xmm2       \n\t"
		"movaps  48(%%rax), %%xmm3       \n\t"
		"                                \n\t"
		"je     .S1_4X4                  \n\t" // if alg==1, jump
		"                                \n\t"
		"                                \n\t"// alg==-1
		"subps  %%xmm8,  %%xmm0           \n\t"
		"subps  %%xmm9,  %%xmm1           \n\t"
		"subps  %%xmm10, %%xmm2           \n\t"
		"subps  %%xmm11, %%xmm3           \n\t"
		"                                \n\t"
		"movaps  %%xmm0, (%%r8)         \n\t"
		"movaps  %%xmm1, 16(%%r8)       \n\t"
		"movaps  %%xmm2, 32(%%r8)       \n\t"
		"movaps  %%xmm3, 48(%%r8)       \n\t"
		"                                \n\t"
		"jmp    .SDONE_4X4               \n\t" // jump to end
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".S1_4X4:                        \n\t" // alg==1
		"                                \n\t"
		"addps  %%xmm0, %%xmm8           \n\t"
		"addps  %%xmm1, %%xmm9           \n\t"
		"addps  %%xmm2, %%xmm10          \n\t"
		"addps  %%xmm3, %%xmm11          \n\t"
		"                                \n\t"
		".S0_4X4:                        \n\t" // alg==0
		"                                \n\t"
		"movaps	%%xmm8,  (%%r8)          \n\t"
		"movaps	%%xmm9,  16(%%r8)        \n\t"
		"movaps	%%xmm10, 32(%%r8)        \n\t"
		"movaps	%%xmm11, 48(%%r8)        \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".SDONE_4X4:                     \n\t"
		"                                \n\t"

		: // output operands (none)
		: // input operands
		  "m" (k_iter),		// %0
		  "m" (k_left),		// %1
		  "m" (A0),			// %2
		  "m" (B),			// %3
		  "m" (C0),			// %4
		  "m" (alg),		// %5
		  "m" (D0)			// %6
		: // register clobber list
		  "rax", "rbx", "rcx", "r8", "r9", "rsi", //"rdx", "rdi", "r8", "r9", "r10", "r11",
		  "xmm0", "xmm1", "xmm2", "xmm3",
		  "xmm4", "xmm5", "xmm6", "xmm7",
		  "xmm8", "xmm9", "xmm10", "xmm11",
		  "xmm12", "xmm13", "xmm14", "xmm15",
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

