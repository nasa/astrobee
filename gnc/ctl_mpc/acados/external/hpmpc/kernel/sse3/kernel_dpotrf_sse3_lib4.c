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

//#include "../../include/block_size.h" // TODO remove when not needed any longer



#include <mmintrin.h>
#include <xmmintrin.h>  // SSE
#include <emmintrin.h>  // SSE2
#include <pmmintrin.h>  // SSE3
/*#include <smmintrin.h>  // SSE4*/
//#include <immintrin.h>  // AVX



// new kernels
void kernel_dpotrf_nt_4x4_lib4_new(int ksub, double *Am, double *Bm, int alg, double *C, double *D, double *inv_diag_D)
	{

	int ki_sub = ksub/4;


	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"movq          %5, %%rax         \n\t" // load address of A
		"movq          %6, %%rbx         \n\t" // load address of B
		"                                \n\t"
		"                                \n\t"
		"xorpd     %%xmm3,  %%xmm3       \n\t" // zero registers
		"movaps    %%xmm3,  %%xmm4       \n\t"
		"movaps    %%xmm3,  %%xmm5       \n\t"
		"movaps    %%xmm3,  %%xmm6       \n\t"
		"movaps    %%xmm3,  %%xmm7       \n\t"
		"movaps    %%xmm3,  %%xmm8       \n\t"
		"movaps    %%xmm3,  %%xmm9       \n\t"
/*		"movaps    %%xmm3, %%xmm10       \n\t"*/
/*		"movaps    %%xmm3, %%xmm11       \n\t"*/
		"movaps    %%xmm3, %%xmm12       \n\t"
		"movaps    %%xmm3, %%xmm13       \n\t"
		"movaps    %%xmm3, %%xmm14       \n\t"
		"movaps    %%xmm3, %%xmm15       \n\t"
		"                                \n\t"
		"                                \n\t"
		"movl   %0, %%esi                \n\t"
		"testl  %%esi, %%esi             \n\t" // check i via logical AND.
		"je     .DPOSTACC_2                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movaps        0(%%rax), %%xmm0  \n\t" // initialize loop by pre-loading elements
		"movaps       16(%%rax), %%xmm1  \n\t" // of a and b.
		"movaps        0(%%rbx), %%xmm2  \n\t"
		"                                \n\t"
		".DLOOPSUB_2:                      \n\t" // main loop 2
		"                                \n\t"
/*		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 0*/
		"movaps       16(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
/*		"subpd   %%xmm4, %%xmm11         \n\t"*/
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps       32(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 1*/
		"movaps       48(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
/*		"subpd   %%xmm4, %%xmm11         \n\t"*/
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps       64(%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps       64(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       80(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 2*/
		"movaps       80(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
/*		"subpd   %%xmm4, %%xmm11         \n\t"*/
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps       96(%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps       96(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps      112(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 3*/
		"movaps      112(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addq      $128, %%rax           \n\t" // A0 += 16
		"                                \n\t"
/*		"subpd   %%xmm4, %%xmm11         \n\t"*/
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addq      $128, %%rbx           \n\t" // B += 16
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps         (%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"decl    %%esi                   \n\t" // i -= 1;
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps         (%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       16(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"jne    .DLOOPSUB_2                \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
/*		"subpd   %%xmm6, %%xmm10         \n\t"*/
		"subpd   %%xmm3, %%xmm14         \n\t"
/*		"subpd   %%xmm4, %%xmm11         \n\t"*/
		"subpd   %%xmm5, %%xmm15         \n\t"
		"                                \n\t"
		"                                \n\t"
		".DPOSTACC_2:                      \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movaps   %%xmm8,  %%xmm0        \n\t"
		"movsd    %%xmm9,  %%xmm8        \n\t"
		"movsd    %%xmm0,  %%xmm9        \n\t"
		"                                \n\t"
/*		"movaps  %%xmm10,  %%xmm0        \n\t"*/
/*		"movsd   %%xmm11, %%xmm10        \n\t"*/
/*		"movsd    %%xmm0, %%xmm11        \n\t"*/
		"                                \n\t"
		"movaps  %%xmm12,  %%xmm0        \n\t"
		"movsd   %%xmm13, %%xmm12        \n\t"
		"movsd    %%xmm0, %%xmm13        \n\t"
		"                                \n\t"
		"movaps  %%xmm14,  %%xmm0        \n\t"
		"movsd   %%xmm15, %%xmm14        \n\t"
		"movsd    %%xmm0, %%xmm15        \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movl      %4, %%esi             \n\t" // alg
		"testl  %%esi, %%esi             \n\t" // check alg via logical AND.
		"je     .DSOLVE_2                  \n\t" // if alg != 0, jump 
		"                                \n\t"
		"                                \n\t"
		"movq   %2, %%rax                \n\t" // load address of C
		"                                \n\t"
		"                                \n\t"
		"movaps  (%%rax),   %%xmm0       \n\t" // load C0
		"movaps  32(%%rax), %%xmm1       \n\t"
/*		"movaps  64(%%rax), %%xmm2       \n\t"*/
/*		"movaps  96(%%rax), %%xmm3       \n\t"*/
		"movaps  16(%%rax), %%xmm4       \n\t" // load C0
		"movaps  48(%%rax), %%xmm5       \n\t"
		"movaps  80(%%rax), %%xmm6       \n\t"
		"movaps 112(%%rax), %%xmm7       \n\t"
		"                                \n\t"
		"                                \n\t"
		"addpd  %%xmm0, %%xmm9           \n\t"
		"addpd  %%xmm1, %%xmm8           \n\t"
/*		"addpd  %%xmm2, %%xmm11          \n\t"*/
/*		"addpd  %%xmm3, %%xmm10          \n\t"*/
		"addpd  %%xmm4, %%xmm13          \n\t"
		"addpd  %%xmm5, %%xmm12          \n\t"
		"addpd  %%xmm6, %%xmm15          \n\t"
		"addpd  %%xmm7, %%xmm14          \n\t"
		"                                \n\t"
		"                                \n\t"
		".DSOLVE_2:                        \n\t"
		"                                \n\t" //  9  8 11 10
		"                                \n\t" // 13 12 15 14
		"                                \n\t"
		"movq   %3, %%rax                \n\t" // load address of inv_diag_D
		"movq   %1, %%rbx                \n\t" // load address of D
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movsd   .DTHR_2(%%rip), %%xmm7    \n\t" // 1e-15
		"movsd   .DONE_2(%%rip), %%xmm6    \n\t" // 1.0
		"                                \n\t"
		"                                \n\t"
		".DA00_2:                          \n\t"
		"movsd   %%xmm9, %%xmm0          \n\t" // a_00
		"ucomisd %%xmm7, %%xmm0          \n\t"
		"jbe     .DZERO0_2                 \n\t"
		"sqrtsd  %%xmm0, %%xmm0          \n\t"
		"movsd   %%xmm6, %%xmm5          \n\t" // 1.0
		"divsd   %%xmm0, %%xmm5          \n\t" // 1.0/a_00
		"movsd   %%xmm5, (%%rax)         \n\t" // inv_diag_D[0]
		"movddup %%xmm5, %%xmm5          \n\t"
		"mulpd   %%xmm5, %%xmm9          \n\t"
		"mulpd   %%xmm5, %%xmm13         \n\t"
		".DA00_END_2:                    \n\t"
		"movaps	 %%xmm9,  (%%rbx)        \n\t"
		"movaps	 %%xmm13, 16(%%rbx)      \n\t"
		"                                \n\t"
		".DA11_2:                          \n\t"
		"movaps  %%xmm9, %%xmm0          \n\t"
		"shufpd  $3, %%xmm0, %%xmm0      \n\t" // a_10
		"movaps  %%xmm0,  %%xmm1         \n\t"
		"mulpd   %%xmm9, %%xmm0          \n\t"
		"mulpd   %%xmm13, %%xmm1         \n\t"
		"subpd   %%xmm0, %%xmm8          \n\t"
		"subpd   %%xmm1, %%xmm12         \n\t"
		"movaps  %%xmm8, %%xmm0          \n\t" // [a_01 a_11]
		"shufpd  $1, %%xmm0, %%xmm0      \n\t" // a_11
		"ucomisd %%xmm7, %%xmm0          \n\t"
		"jbe     .DZERO1_2                 \n\t"
		"sqrtsd  %%xmm0, %%xmm0          \n\t"
		"movsd   %%xmm6, %%xmm5          \n\t" // 1.0
		"divsd   %%xmm0, %%xmm5          \n\t" // 1.0/a_11
		"movsd   %%xmm5, 8(%%rax)        \n\t" // inv_diag_D[1]
		"movddup %%xmm5, %%xmm5          \n\t"
		"mulpd   %%xmm5, %%xmm8          \n\t"
		"mulpd   %%xmm5, %%xmm12         \n\t"
		".DA11_END_2:                    \n\t"
		"movhpd	 %%xmm8,  40(%%rbx)      \n\t"
		"movaps	 %%xmm12, 48(%%rbx)      \n\t"
		"                                \n\t"
		".DA22_2:                          \n\t"
		"movddup %%xmm13, %%xmm0         \n\t" // a_20
		"movddup %%xmm12, %%xmm1         \n\t" // a_21
		"mulpd   %%xmm13, %%xmm0         \n\t"
		"mulpd   %%xmm12, %%xmm1         \n\t"
		"subpd   %%xmm0, %%xmm15         \n\t"
		"subpd   %%xmm1, %%xmm15         \n\t"
		"movsd   %%xmm15, %%xmm0         \n\t" // a_22
		"ucomisd %%xmm7, %%xmm0          \n\t"
		"jbe     .DZERO2_2                 \n\t"
		"sqrtsd  %%xmm0, %%xmm0          \n\t"
		"movsd   %%xmm6, %%xmm5          \n\t" // 1.0
		"divsd   %%xmm0, %%xmm5          \n\t" // 1.0/a_11
		"movsd   %%xmm5, 16(%%rax)       \n\t" // inv_diag_D[2]
		"movddup %%xmm5, %%xmm5          \n\t"
		"mulpd   %%xmm5, %%xmm15         \n\t"
		".DA22_END_2:                    \n\t"
		"movaps	 %%xmm15, 80(%%rbx)      \n\t"
		"                                \n\t"
		".DA33_2:                          \n\t"
		"movaps  %%xmm13, %%xmm0         \n\t"
		"movaps  %%xmm12, %%xmm1         \n\t"
		"movaps  %%xmm15, %%xmm2         \n\t"
		"shufpd  $3, %%xmm0, %%xmm0      \n\t" // a_30
		"shufpd  $3, %%xmm1, %%xmm1      \n\t" // a_31
		"shufpd  $3, %%xmm2, %%xmm2      \n\t" // a_32
		"mulpd   %%xmm13, %%xmm0         \n\t"
		"mulpd   %%xmm12, %%xmm1         \n\t"
		"mulpd   %%xmm15, %%xmm2         \n\t"
		"subpd   %%xmm0, %%xmm14         \n\t"
		"subpd   %%xmm1, %%xmm14         \n\t"
		"subpd   %%xmm2, %%xmm14         \n\t"
		"movaps  %%xmm14, %%xmm0         \n\t" // [a_23 a_33]
		"shufpd  $1, %%xmm0, %%xmm0      \n\t" // a_33
		"ucomisd %%xmm7, %%xmm0          \n\t"
		"jbe     .DZERO3_2                 \n\t"
		"sqrtsd  %%xmm0, %%xmm0          \n\t"
		"movsd   %%xmm0, 120(%%rbx)      \n\t"
		"divsd   %%xmm0, %%xmm6          \n\t" // 1.0/a_11
		"movsd   %%xmm6, 24(%%rax)       \n\t" // inv_diag_D[3]
		"                                \n\t"
		"jmp    .DEND_2                    \n\t"
		"                                \n\t"
		"                                \n\t"
		".DZERO0_2:                        \n\t"
		"xorpd  %%xmm9,  %%xmm9          \n\t"
		"movaps %%xmm9,  %%xmm13         \n\t"
		"movsd  %%xmm9,  (%%rax)         \n\t" // inv_diag_D[0]
		"jmp    .DA00_END_2              \n\t"
		"                                \n\t"
		".DZERO1_2:                        \n\t"
		"xorpd  %%xmm8,  %%xmm8          \n\t"
		"movaps %%xmm8,  %%xmm12         \n\t"
		"movsd  %%xmm8,  8(%%rax)       \n\t" // inv_diag_D[1]
		"jmp    .DA11_END_2              \n\t"
		"                                \n\t"
		".DZERO2_2:                        \n\t"
		"xorpd  %%xmm15, %%xmm15         \n\t"
		"movsd  %%xmm15, 16(%%rax)       \n\t" // inv_diag_D[2]
		"jmp    .DA22_END_2              \n\t"
		"                                \n\t"
		".DZERO3_2:                        \n\t"
		"xorpd  %%xmm14, %%xmm14         \n\t"
		"movsd  %%xmm14, 120(%%rbx)      \n\t"
		"movsd  %%xmm14, 24(%%rax)       \n\t" // inv_diag_D[3]
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"jmp    .DEND_2                    \n\t"
		".align 16                       \n\t"
		".DONE_2:                          \n\t"
		".long  0                        \n\t"
		".long  1072693248               \n\t"
		".long  0                        \n\t"
		".long  0                        \n\t"
		".align 16                       \n\t"
		".DTHR_2:                          \n\t"
		".long  2665960982               \n\t"
		".long  1020396463               \n\t"
		".long  0                        \n\t"
		".long  0                        \n\t"
		"                                \n\t"
		"                                \n\t"
		".DEND_2:                          \n\t"
		"                                \n\t"
		: // output operands (none)
		: // input operands
		  "m" (ki_sub),		// %0
		  "m" (C),			// %1
		  "m" (D),			// %2
		  "m" (inv_diag_D),	// %3
		  "m" (alg),		// %4
		  "m" (Am),			// %5
		  "m" (Bm) 			// %6
		: // register clobber list
		  "rax", "rbx", "rcx", "rdx", "rsi", //"rdx", //"rdi", "r8", "r9", "r10", "r11",
		  "xmm0", "xmm1", "xmm2", "xmm3",
		  "xmm4", "xmm5", "xmm6", "xmm7",
		  "xmm8", "xmm9", "xmm10", "xmm11",
		  "xmm12", "xmm13", "xmm14", "xmm15",
		  "memory"
	);
}



void kernel_dsyrk_dpotrf_nt_4x4_lib4_new(int kadd, double *Ap, double *Bp, int ksub, double *Am, double *Bm, int alg, double *C, double *D, double *inv_diag_D)
	{

	int ki_add = kadd/4;
	int kl_add = kadd%4;
	int ki_sub = ksub/4;


	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"movq          %3, %%rax         \n\t" // load address of A
		"movq          %4, %%rbx         \n\t" // load address of B
		"                                \n\t"
		"                                \n\t"
		"xorpd     %%xmm3,  %%xmm3       \n\t" // zero registers
		"movaps    %%xmm3,  %%xmm4       \n\t"
		"movaps    %%xmm3,  %%xmm5       \n\t"
		"movaps    %%xmm3,  %%xmm6       \n\t"
		"movaps    %%xmm3,  %%xmm7       \n\t"
		"movaps    %%xmm3,  %%xmm8       \n\t"
		"movaps    %%xmm3,  %%xmm9       \n\t"
/*		"movaps    %%xmm3, %%xmm10       \n\t"*/
/*		"movaps    %%xmm3, %%xmm11       \n\t"*/
		"movaps    %%xmm3, %%xmm12       \n\t"
		"movaps    %%xmm3, %%xmm13       \n\t"
		"movaps    %%xmm3, %%xmm14       \n\t"
		"movaps    %%xmm3, %%xmm15       \n\t"
		"                                \n\t"
		"                                \n\t"
		"movl      %0, %%esi             \n\t" // ki_add
		"movl      %1, %%ecx             \n\t" // kl_add
		"addl      %%esi, %%ecx          \n\t" // ki_add + kl_add
		"testl  %%ecx, %%ecx             \n\t" 
		"je     .DCONSIDERSUB_1          \n\t" // kadd = 0
		"                                \n\t"
		"                                \n\t"
		"movaps        0(%%rax), %%xmm0  \n\t" // initialize loop by pre-loading elements
		"movaps       16(%%rax), %%xmm1  \n\t" // of a and b.
		"movaps        0(%%rbx), %%xmm2  \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"testl  %%esi, %%esi             \n\t" // logical AND
		"je     .DCONSIDERADD_1          \n\t" // if ki_add == 0, jump
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DLOOPADD_1:                    \n\t" // MAIN LOOP add
		"                                \n\t"
/*		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 0*/
		"movaps       16(%%rbx), %%xmm6  \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
/*		"addpd   %%xmm4, %%xmm11         \n\t"*/
		"addpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
		"addpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps       32(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 1*/
		"movaps       48(%%rbx), %%xmm6  \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
/*		"addpd   %%xmm4, %%xmm11         \n\t"*/
		"addpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       64(%%rbx), %%xmm2  \n\t"
		"addpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps       64(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       80(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 2*/
		"movaps       80(%%rbx), %%xmm6  \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
/*		"addpd   %%xmm4, %%xmm11         \n\t"*/
		"addpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       96(%%rbx), %%xmm2  \n\t"
		"addpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps       96(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps      112(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 3*/
		"movaps      112(%%rbx), %%xmm6  \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addq      $128, %%rax           \n\t" // A0 += 16
		"                                \n\t"
/*		"addpd   %%xmm4, %%xmm11         \n\t"*/
		"addpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addq      $128, %%rbx           \n\t" // B += 16
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps         (%%rbx), %%xmm2  \n\t"
		"addpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"decl    %%esi                   \n\t" // i -= 1;
		"addpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps         (%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       16(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"jne    .DLOOPADD_1                \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERADD_1:                  \n\t"
		"                                \n\t"
		"movl      %1, %%esi             \n\t" // i = k_left;
		"testl  %%esi, %%esi             \n\t" // check i via logical AND.
		"je     .DCONSIDERSUB_1            \n\t" // if i == 0, we're done; jump to end.
		"                                \n\t" // else, we prepare to enter k_left loop.
		"                                \n\t"
		"                                \n\t"
		".DLOOPLEFT_1:                     \n\t" // EDGE LOOP
		"                                \n\t"
/*		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 0*/
		"movaps       16(%%rbx), %%xmm6  \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
/*		"addpd   %%xmm4, %%xmm11         \n\t"*/
		"addpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
		"addpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps       32(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"addq          $32, %%rax        \n\t" // A += 4
		"addq          $32, %%rbx        \n\t" // B += 4
		"                                \n\t"
		"                                \n\t"
		"decl    %%esi                   \n\t" // i -= 1;
		"jne    .DLOOPLEFT_1               \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERSUB_1:                  \n\t" // padd
		"                                \n\t"
		"                                \n\t"
/*		"addpd   %%xmm6, %%xmm10         \n\t"*/
		"addpd   %%xmm3, %%xmm14         \n\t"
/*		"addpd   %%xmm4, %%xmm11         \n\t"*/
		"addpd   %%xmm5, %%xmm15         \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERSUB_NOCLEAN_1:          \n\t" // padd
		"                                \n\t"
		"                                \n\t"
		"xorpd     %%xmm3,  %%xmm3       \n\t"
		"movaps    %%xmm3,  %%xmm4       \n\t"
		"movaps    %%xmm3,  %%xmm5       \n\t"
		"movaps    %%xmm3,  %%xmm6       \n\t"
		"                                \n\t"
		"                                \n\t"
		"movl   %2, %%esi                \n\t"
		"testl  %%esi, %%esi             \n\t" // check i via logical AND.
		"je     .DPOSTACC_1                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movq   %9,  %%rax               \n\t"
		"movq   %10, %%rbx               \n\t"
		"                                \n\t"
		"movaps        0(%%rax), %%xmm0  \n\t" // initialize loop by pre-loading elements
		"movaps       16(%%rax), %%xmm1  \n\t" // of a and b.
		"movaps        0(%%rbx), %%xmm2  \n\t"
		"                                \n\t"
		"                                \n\t"
		".DLOOPSUB_1:                      \n\t" // main loop 2
		"                                \n\t"
/*		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 0*/
		"movaps       16(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
/*		"subpd   %%xmm4, %%xmm11         \n\t"*/
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps       32(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 1*/
		"movaps       48(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
/*		"subpd   %%xmm4, %%xmm11         \n\t"*/
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps       64(%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps       64(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       80(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 2*/
		"movaps       80(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
/*		"subpd   %%xmm4, %%xmm11         \n\t"*/
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps       96(%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps       96(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps      112(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 3*/
		"movaps      112(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addq      $128, %%rax           \n\t" // A0 += 16
		"                                \n\t"
/*		"subpd   %%xmm4, %%xmm11         \n\t"*/
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addq      $128, %%rbx           \n\t" // B += 16
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps         (%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"decl    %%esi                   \n\t" // i -= 1;
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps         (%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       16(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"jne    .DLOOPSUB_1                \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
/*		"subpd   %%xmm6, %%xmm10         \n\t"*/
		"subpd   %%xmm3, %%xmm14         \n\t"
/*		"subpd   %%xmm4, %%xmm11         \n\t"*/
		"subpd   %%xmm5, %%xmm15         \n\t"
		"                                \n\t"
		"                                \n\t"
		".DPOSTACC_1:                      \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movaps   %%xmm8,  %%xmm0        \n\t"
		"movsd    %%xmm9,  %%xmm8        \n\t"
		"movsd    %%xmm0,  %%xmm9        \n\t"
		"                                \n\t"
/*		"movaps  %%xmm10,  %%xmm0        \n\t"*/
/*		"movsd   %%xmm11, %%xmm10        \n\t"*/
/*		"movsd    %%xmm0, %%xmm11        \n\t"*/
		"                                \n\t"
		"movaps  %%xmm12,  %%xmm0        \n\t"
		"movsd   %%xmm13, %%xmm12        \n\t"
		"movsd    %%xmm0, %%xmm13        \n\t"
		"                                \n\t"
		"movaps  %%xmm14,  %%xmm0        \n\t"
		"movsd   %%xmm15, %%xmm14        \n\t"
		"movsd    %%xmm0, %%xmm15        \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movl      %8, %%esi             \n\t" // alg
		"testl  %%esi, %%esi             \n\t" // check alg via logical AND.
		"je     .DSOLVE_1                  \n\t" // if alg != 0, jump 
		"                                \n\t"
		"                                \n\t"
		"movq   %5, %%rax                \n\t" // load address of C
		"                                \n\t"
		"                                \n\t"
		"movaps  (%%rax),   %%xmm0       \n\t" // load C0
		"movaps  32(%%rax), %%xmm1       \n\t"
/*		"movaps  64(%%rax), %%xmm2       \n\t"*/
/*		"movaps  96(%%rax), %%xmm3       \n\t"*/
		"movaps  16(%%rax), %%xmm4       \n\t" // load C0
		"movaps  48(%%rax), %%xmm5       \n\t"
		"movaps  80(%%rax), %%xmm6       \n\t"
		"movaps 112(%%rax), %%xmm7       \n\t"
		"                                \n\t"
		"                                \n\t"
		"addpd  %%xmm0, %%xmm9           \n\t"
		"addpd  %%xmm1, %%xmm8           \n\t"
/*		"addpd  %%xmm2, %%xmm11          \n\t"*/
/*		"addpd  %%xmm3, %%xmm10          \n\t"*/
		"addpd  %%xmm4, %%xmm13          \n\t"
		"addpd  %%xmm5, %%xmm12          \n\t"
		"addpd  %%xmm6, %%xmm15          \n\t"
		"addpd  %%xmm7, %%xmm14          \n\t"
		"                                \n\t"
		"                                \n\t"
		".DSOLVE_1:                        \n\t"
		"                                \n\t" //  9  8 11 10
		"                                \n\t" // 13 12 15 14
		"                                \n\t"
		"movq   %7, %%rax                \n\t" // load address of inv_diag_D
		"movq   %6, %%rbx                \n\t" // load address of D
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movsd   .DTHR_1(%%rip), %%xmm7    \n\t" // 1e-15
		"movsd   .DONE_1(%%rip), %%xmm6    \n\t" // 1.0
		"                                \n\t"
		"                                \n\t"
		".DA00_1:                          \n\t"
		"movsd   %%xmm9, %%xmm0          \n\t" // a_00
		"ucomisd %%xmm7, %%xmm0          \n\t"
		"jbe     .DZERO0_1                 \n\t"
		"sqrtsd  %%xmm0, %%xmm0          \n\t"
		"movsd   %%xmm6, %%xmm5          \n\t" // 1.0
		"divsd   %%xmm0, %%xmm5          \n\t" // 1.0/a_00
		"movsd   %%xmm5, (%%rax)         \n\t" // inv_diag_D[0]
		"movddup %%xmm5, %%xmm5          \n\t"
		"mulpd   %%xmm5, %%xmm9          \n\t"
		"mulpd   %%xmm5, %%xmm13         \n\t"
		".DA00_END_1:                    \n\t"
		"movaps	 %%xmm9,  (%%rbx)        \n\t"
		"movaps	 %%xmm13, 16(%%rbx)      \n\t"
		"                                \n\t"
		".DA11_1:                          \n\t"
		"movaps  %%xmm9, %%xmm0          \n\t"
		"shufpd  $3, %%xmm0, %%xmm0      \n\t" // a_10
		"movaps  %%xmm0,  %%xmm1         \n\t"
		"mulpd   %%xmm9, %%xmm0          \n\t"
		"mulpd   %%xmm13, %%xmm1         \n\t"
		"subpd   %%xmm0, %%xmm8          \n\t"
		"subpd   %%xmm1, %%xmm12         \n\t"
		"movaps  %%xmm8, %%xmm0          \n\t" // [a_01 a_11]
		"shufpd  $1, %%xmm0, %%xmm0      \n\t" // a_11
		"ucomisd %%xmm7, %%xmm0          \n\t"
		"jbe     .DZERO1_1                 \n\t"
		"sqrtsd  %%xmm0, %%xmm0          \n\t"
		"movsd   %%xmm6, %%xmm5          \n\t" // 1.0
		"divsd   %%xmm0, %%xmm5          \n\t" // 1.0/a_11
		"movsd   %%xmm5, 8(%%rax)        \n\t" // inv_diag_D[1]
		"movddup %%xmm5, %%xmm5          \n\t"
		"mulpd   %%xmm5, %%xmm8          \n\t"
		"mulpd   %%xmm5, %%xmm12         \n\t"
		".DA11_END_1:                    \n\t"
		"movhpd	 %%xmm8,  40(%%rbx)      \n\t"
		"movaps	 %%xmm12, 48(%%rbx)      \n\t"
		"                                \n\t"
		".DA22_1:                          \n\t"
		"movddup %%xmm13, %%xmm0         \n\t" // a_20
		"movddup %%xmm12, %%xmm1         \n\t" // a_21
		"mulpd   %%xmm13, %%xmm0         \n\t"
		"mulpd   %%xmm12, %%xmm1         \n\t"
		"subpd   %%xmm0, %%xmm15         \n\t"
		"subpd   %%xmm1, %%xmm15         \n\t"
		"movsd   %%xmm15, %%xmm0         \n\t" // a_22
		"ucomisd %%xmm7, %%xmm0          \n\t"
		"jbe     .DZERO2_1                 \n\t"
		"sqrtsd  %%xmm0, %%xmm0          \n\t"
		"movsd   %%xmm6, %%xmm5          \n\t" // 1.0
		"divsd   %%xmm0, %%xmm5          \n\t" // 1.0/a_11
		"movsd   %%xmm5, 16(%%rax)       \n\t" // inv_diag_D[2]
		"movddup %%xmm5, %%xmm5          \n\t"
		"mulpd   %%xmm5, %%xmm15         \n\t"
		".DA22_END_1:                    \n\t"
		"movaps	 %%xmm15, 80(%%rbx)      \n\t"
		"                                \n\t"
		".DA33_1:                          \n\t"
		"movaps  %%xmm13, %%xmm0         \n\t"
		"movaps  %%xmm12, %%xmm1         \n\t"
		"movaps  %%xmm15, %%xmm2         \n\t"
		"shufpd  $3, %%xmm0, %%xmm0      \n\t" // a_30
		"shufpd  $3, %%xmm1, %%xmm1      \n\t" // a_31
		"shufpd  $3, %%xmm2, %%xmm2      \n\t" // a_32
		"mulpd   %%xmm13, %%xmm0         \n\t"
		"mulpd   %%xmm12, %%xmm1         \n\t"
		"mulpd   %%xmm15, %%xmm2         \n\t"
		"subpd   %%xmm0, %%xmm14         \n\t"
		"subpd   %%xmm1, %%xmm14         \n\t"
		"subpd   %%xmm2, %%xmm14         \n\t"
		"movaps  %%xmm14, %%xmm0         \n\t" // [a_23 a_33]
		"shufpd  $1, %%xmm0, %%xmm0      \n\t" // a_33
		"ucomisd %%xmm7, %%xmm0          \n\t"
		"jbe     .DZERO3_1                 \n\t"
		"sqrtsd  %%xmm0, %%xmm0          \n\t"
		"movsd   %%xmm0, 120(%%rbx)      \n\t"
		"divsd   %%xmm0, %%xmm6          \n\t" // 1.0/a_11
		"movsd   %%xmm6, 24(%%rax)       \n\t" // inv_diag_D[3]
		"                                \n\t"
		"jmp    .DEND_1                    \n\t"
		"                                \n\t"
		"                                \n\t"
		".DZERO0_1:                        \n\t"
		"xorpd  %%xmm9,  %%xmm9          \n\t"
		"movaps %%xmm9,  %%xmm13         \n\t"
		"movsd  %%xmm9,  (%%rax)         \n\t" // inv_diag_D[0]
		"jmp    .DA00_END_1              \n\t"
		"                                \n\t"
		".DZERO1_1:                        \n\t"
		"xorpd  %%xmm8,  %%xmm8          \n\t"
		"movaps %%xmm8,  %%xmm12         \n\t"
		"movsd  %%xmm8,  8(%%rax)       \n\t" // inv_diag_D[1]
		"jmp    .DA11_END_1              \n\t"
		"                                \n\t"
		".DZERO2_1:                        \n\t"
		"xorpd  %%xmm15, %%xmm15         \n\t"
		"movsd  %%xmm15, 16(%%rax)       \n\t" // inv_diag_D[2]
		"jmp    .DA22_END_1              \n\t"
		"                                \n\t"
		".DZERO3_1:                        \n\t"
		"xorpd  %%xmm14, %%xmm14         \n\t"
		"movsd  %%xmm14, 120(%%rbx)      \n\t"
		"movsd  %%xmm14, 24(%%rax)       \n\t" // inv_diag_D[3]
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"jmp    .DEND_1                    \n\t"
		".align 16                       \n\t"
		".DONE_1:                          \n\t"
		".long  0                        \n\t"
		".long  1072693248               \n\t"
		".long  0                        \n\t"
		".long  0                        \n\t"
		".align 16                       \n\t"
		".DTHR_1:                          \n\t"
		".long  2665960982               \n\t"
		".long  1020396463               \n\t"
		".long  0                        \n\t"
		".long  0                        \n\t"
		"                                \n\t"
		"                                \n\t"
		".DEND_1:                          \n\t"
		"                                \n\t"
		: // output operands (none)
		: // input operands
		  "m" (ki_add),		// %0
		  "m" (kl_add),		// %1
		  "m" (ki_sub),		// %2
		  "m" (Ap),			// %3
		  "m" (Bp),			// %4
		  "m" (C),			// %5
		  "m" (D),			// %6
		  "m" (inv_diag_D),	// %7
		  "m" (alg),		// %8
		  "m" (Am),			// %9
		  "m" (Bm) 			// %10
		: // register clobber list
		  "rax", "rbx", "rcx", "rdx", "rsi", //"rdx", //"rdi", "r8", "r9", "r10", "r11",
		  "xmm0", "xmm1", "xmm2", "xmm3",
		  "xmm4", "xmm5", "xmm6", "xmm7",
		  "xmm8", "xmm9", "xmm10", "xmm11",
		  "xmm12", "xmm13", "xmm14", "xmm15",
		  "memory"
	);
}



void kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(int km, int kn, int kadd, int tri_A, double *Ap, double *Bp, int ksub, double *Am, double *Bm, int alg, double *C, double *D, double *inv_diag_D)
	{

	int ki_add = kadd/4;
	int kl_add = kadd%4;
	int ki_sub = ksub/4;


	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"movq          %3, %%rax         \n\t" // load address of A
		"movq          %4, %%rbx         \n\t" // load address of B
		"                                \n\t"
		"                                \n\t"
		"xorpd     %%xmm3,  %%xmm3       \n\t" // zero registers
		"movaps    %%xmm3,  %%xmm4       \n\t"
		"movaps    %%xmm3,  %%xmm5       \n\t"
		"movaps    %%xmm3,  %%xmm6       \n\t"
		"movaps    %%xmm3,  %%xmm7       \n\t"
		"movaps    %%xmm3,  %%xmm8       \n\t"
		"movaps    %%xmm3,  %%xmm9       \n\t"
/*		"movaps    %%xmm3, %%xmm10       \n\t"*/
/*		"movaps    %%xmm3, %%xmm11       \n\t"*/
		"movaps    %%xmm3, %%xmm12       \n\t"
		"movaps    %%xmm3, %%xmm13       \n\t"
		"movaps    %%xmm3, %%xmm14       \n\t"
		"movaps    %%xmm3, %%xmm15       \n\t"
		"                                \n\t"
		"                                \n\t"
		"movl      %0, %%esi             \n\t" // ki_add
		"movl      %1, %%ecx             \n\t" // kl_add
		"addl      %%esi, %%ecx          \n\t" // ki_add + kl_add
		"testl  %%ecx, %%ecx             \n\t" 
		"je     .DCONSIDERSUB_0            \n\t" // kadd = 0
		"                                \n\t"
		"                                \n\t"
		"movaps        0(%%rax), %%xmm0  \n\t" // initialize loop by pre-loading elements
		"movaps       16(%%rax), %%xmm1  \n\t" // of a and b.
		"movaps        0(%%rbx), %%xmm2  \n\t"
		"                                \n\t"
		"                                \n\t"
		"movl      %11, %%ecx            \n\t"
		"cmpl      $1,  %%ecx            \n\t"
		"jne       .DCONSIDERLOOPADD_0     \n\t"
		"                                \n\t"
		"                                \n\t"
		"testl  %%esi, %%esi             \n\t" // logical AND
		"je     .DTRIADD_0                 \n\t" // if ki_add == 0, jump
		"                                \n\t"
		"                                \n\t"
//		"movaps       16(%%rbx), %%xmm6  \n\t"
//		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulsd   %%xmm0, %%xmm2          \n\t"
		"                                \n\t"
//		"mulsd   %%xmm0, %%xmm7          \n\t"
		"                                \n\t"
		"addsd   %%xmm2, %%xmm8          \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
//		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
//		"mulsd   %%xmm0, %%xmm6          \n\t"
		"                                \n\t"
//		"addsd   %%xmm7, %%xmm9          \n\t"
//		"mulsd   %%xmm0, %%xmm4          \n\t"
		"movaps       32(%%rax), %%xmm0  \n\t"
//		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
//		"addsd   %%xmm6, %%xmm10         \n\t" // iteration 1
//		"movaps       48(%%rbx), %%xmm6  \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"                                \n\t"
//		"addsd   %%xmm4, %%xmm11         \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       64(%%rbx), %%xmm2  \n\t"
//		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
//		"mulpd   %%xmm0, %%xmm6          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
//		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       64(%%rax), %%xmm0  \n\t"
		"movaps       80(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
//		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 2
		"movaps       80(%%rbx), %%xmm6  \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulsd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
//		"addpd   %%xmm4, %%xmm11         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulsd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       96(%%rbx), %%xmm2  \n\t"
		"addsd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
//		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulsd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addsd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
//		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       96(%%rax), %%xmm0  \n\t"
		"mulsd   %%xmm1, %%xmm5          \n\t"
		"movaps      112(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
//		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 3
		"movaps      112(%%rbx), %%xmm6  \n\t"
		"addsd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"addq      $128, %%rax           \n\t" // A0 += 16
		"                                \n\t"
//		"addpd   %%xmm4, %%xmm11         \n\t"
		"addsd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"addq      $128, %%rbx           \n\t" // B += 16
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps         (%%rbx), %%xmm2  \n\t"
		"addpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
//		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"decl    %%esi                   \n\t" // i -= 1;
		"addpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
//		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps         (%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       16(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"jmp     .DCONSIDERLOOPADD_0       \n\t"
		"                                \n\t"
		"                                \n\t"
		".DTRIADD_0:                       \n\t"
		"                                \n\t"
		"movl    %1, %%esi               \n\t"
		"                                \n\t"
		"                                \n\t"
//		"movaps       16(%%rbx), %%xmm6  \n\t" // iteration 0
//		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulsd   %%xmm0, %%xmm2          \n\t"
		"addsd   %%xmm2, %%xmm8          \n\t"
		"                                \n\t"
//		"mulsd   %%xmm0, %%xmm7          \n\t"
//		"addsd   %%xmm7, %%xmm9          \n\t"
		"                                \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
//		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
//		"mulsd   %%xmm0, %%xmm6          \n\t"
//		"addsd   %%xmm6, %%xmm10         \n\t"
		"                                \n\t"
//		"mulsd   %%xmm0, %%xmm4          \n\t"
//		"addsd   %%xmm4, %%xmm11         \n\t"
		"movaps       32(%%rax), %%xmm0  \n\t"
//		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
//		"cmpl    $1, %%esi               \n\t"
//		"jle    .DCONSIDERSUB_NOCLEAN_0    \n\t" //
		"cmpl    $2, %%esi               \n\t"
		"jl     .DCONSIDERSUB_NOCLEAN_0    \n\t" //
		"                                \n\t"
		"                                \n\t"
//		"movaps       48(%%rbx), %%xmm6  \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"                                \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"                                \n\t"
		"movaps       64(%%rbx), %%xmm2  \n\t"
//		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
//		"mulpd   %%xmm0, %%xmm6          \n\t"
//		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 1
		"                                \n\t"
//		"mulpd   %%xmm0, %%xmm4          \n\t"
//		"addpd   %%xmm4, %%xmm11         \n\t"
		"movaps       64(%%rax), %%xmm0  \n\t"
		"movaps       80(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
//		"cmpl    $2, %%esi               \n\t"
//		"jle    .DCONSIDERSUB_NOCLEAN    \n\t" //
		"je     .DCONSIDERSUB_NOCLEAN_0    \n\t" //
		"                                \n\t"
		"                                \n\t"
		"movaps       80(%%rbx), %%xmm6  \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulsd   %%xmm1, %%xmm3          \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"addsd   %%xmm3, %%xmm12         \n\t"
		"                                \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulsd   %%xmm1, %%xmm5          \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addsd   %%xmm5, %%xmm13         \n\t"
		"                                \n\t"
//		"movaps       96(%%rbx), %%xmm2  \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
//		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulsd   %%xmm1, %%xmm3          \n\t"
//		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 2
		"addsd   %%xmm3, %%xmm14         \n\t"
		"                                \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
//		"mulpd   %%xmm0, %%xmm4          \n\t"
//		"movaps       96(%%rax), %%xmm0  \n\t"
		"mulsd   %%xmm1, %%xmm5          \n\t"
//		"movaps      112(%%rax), %%xmm1  \n\t"
//		"addpd   %%xmm4, %%xmm11         \n\t"
		"addsd   %%xmm5, %%xmm15         \n\t"
		"                                \n\t"
		"                                \n\t"
		"jmp    .DCONSIDERSUB_NOCLEAN_0    \n\t" //
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERLOOPADD_0:              \n\t" // MAIN LOOP add
		"                                \n\t"
		"                                \n\t"
		"testl  %%esi, %%esi             \n\t" // logical AND
		"je     .DCONSIDERADD_0            \n\t" // if ki_add == 0, jump
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DLOOPADD_0:                      \n\t" // MAIN LOOP add
		"                                \n\t"
/*		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 0*/
		"movaps       16(%%rbx), %%xmm6  \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
/*		"addpd   %%xmm4, %%xmm11         \n\t"*/
		"addpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
		"addpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps       32(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 1*/
		"movaps       48(%%rbx), %%xmm6  \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
/*		"addpd   %%xmm4, %%xmm11         \n\t"*/
		"addpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       64(%%rbx), %%xmm2  \n\t"
		"addpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps       64(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       80(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 2*/
		"movaps       80(%%rbx), %%xmm6  \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
/*		"addpd   %%xmm4, %%xmm11         \n\t"*/
		"addpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       96(%%rbx), %%xmm2  \n\t"
		"addpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps       96(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps      112(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 3*/
		"movaps      112(%%rbx), %%xmm6  \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addq      $128, %%rax           \n\t" // A0 += 16
		"                                \n\t"
/*		"addpd   %%xmm4, %%xmm11         \n\t"*/
		"addpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addq      $128, %%rbx           \n\t" // B += 16
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps         (%%rbx), %%xmm2  \n\t"
		"addpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"decl    %%esi                   \n\t" // i -= 1;
		"addpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps         (%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       16(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"jne    .DLOOPADD_0                \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERADD_0:                  \n\t"
		"                                \n\t"
		"movl      %1, %%esi             \n\t" // i = k_left;
		"testl  %%esi, %%esi             \n\t" // check i via logical AND.
		"je     .DCONSIDERSUB_0            \n\t" // if i == 0, we're done; jump to end.
		"                                \n\t" // else, we prepare to enter k_left loop.
		"                                \n\t"
		"                                \n\t"
		".DLOOPLEFT_0:                     \n\t" // EDGE LOOP
		"                                \n\t"
/*		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 0*/
		"movaps       16(%%rbx), %%xmm6  \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
/*		"addpd   %%xmm4, %%xmm11         \n\t"*/
		"addpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
		"addpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps       32(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"addq          $32, %%rax        \n\t" // A += 4
		"addq          $32, %%rbx        \n\t" // B += 4
		"                                \n\t"
		"                                \n\t"
		"decl    %%esi                   \n\t" // i -= 1;
		"jne    .DLOOPLEFT_0               \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERSUB_0:                  \n\t" // padd
		"                                \n\t"
		"                                \n\t"
/*		"addpd   %%xmm6, %%xmm10         \n\t"*/
		"addpd   %%xmm3, %%xmm14         \n\t"
/*		"addpd   %%xmm4, %%xmm11         \n\t"*/
		"addpd   %%xmm5, %%xmm15         \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERSUB_NOCLEAN_0:          \n\t" // padd
		"                                \n\t"
		"                                \n\t"
		"xorpd     %%xmm3,  %%xmm3       \n\t"
		"movaps    %%xmm3,  %%xmm4       \n\t"
		"movaps    %%xmm3,  %%xmm5       \n\t"
		"movaps    %%xmm3,  %%xmm6       \n\t"
		"                                \n\t"
		"                                \n\t"
		"movl   %2, %%esi                \n\t"
		"testl  %%esi, %%esi             \n\t" // check i via logical AND.
		"je     .DPOSTACC_0                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movq   %9,  %%rax               \n\t"
		"movq   %10, %%rbx               \n\t"
		"                                \n\t"
		"movaps        0(%%rax), %%xmm0  \n\t" // initialize loop by pre-loading elements
		"movaps       16(%%rax), %%xmm1  \n\t" // of a and b.
		"movaps        0(%%rbx), %%xmm2  \n\t"
		"                                \n\t"
		"                                \n\t"
		".DLOOPSUB_0:                      \n\t" // main loop 2
		"                                \n\t"
/*		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 0*/
		"movaps       16(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
/*		"subpd   %%xmm4, %%xmm11         \n\t"*/
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps       32(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 1*/
		"movaps       48(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
/*		"subpd   %%xmm4, %%xmm11         \n\t"*/
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps       64(%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps       64(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       80(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 2*/
		"movaps       80(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
/*		"subpd   %%xmm4, %%xmm11         \n\t"*/
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps       96(%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps       96(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps      112(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 3*/
		"movaps      112(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addq      $128, %%rax           \n\t" // A0 += 16
		"                                \n\t"
/*		"subpd   %%xmm4, %%xmm11         \n\t"*/
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addq      $128, %%rbx           \n\t" // B += 16
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps         (%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"decl    %%esi                   \n\t" // i -= 1;
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps         (%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       16(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"jne    .DLOOPSUB_0                \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
/*		"subpd   %%xmm6, %%xmm10         \n\t"*/
		"subpd   %%xmm3, %%xmm14         \n\t"
/*		"subpd   %%xmm4, %%xmm11         \n\t"*/
		"subpd   %%xmm5, %%xmm15         \n\t"
		"                                \n\t"
		"                                \n\t"
		".DPOSTACC_0:                      \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movaps   %%xmm8,  %%xmm0        \n\t"
		"movsd    %%xmm9,  %%xmm8        \n\t"
		"movsd    %%xmm0,  %%xmm9        \n\t"
		"                                \n\t"
/*		"movaps  %%xmm10,  %%xmm0        \n\t"*/
/*		"movsd   %%xmm11, %%xmm10        \n\t"*/
/*		"movsd    %%xmm0, %%xmm11        \n\t"*/
		"                                \n\t"
		"movaps  %%xmm12,  %%xmm0        \n\t"
		"movsd   %%xmm13, %%xmm12        \n\t"
		"movsd    %%xmm0, %%xmm13        \n\t"
		"                                \n\t"
		"movaps  %%xmm14,  %%xmm0        \n\t"
		"movsd   %%xmm15, %%xmm14        \n\t"
		"movsd    %%xmm0, %%xmm15        \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movl      %8, %%esi             \n\t" // alg
		"testl  %%esi, %%esi             \n\t" // check alg via logical AND.
		"je     .DSOLVE_0                  \n\t" // if alg != 0, jump 
		"                                \n\t"
		"                                \n\t"
		"movq   %5, %%rax                \n\t" // load address of C
		"                                \n\t"
		"                                \n\t"
		"movaps  (%%rax),   %%xmm0       \n\t" // load C0
		"movaps  32(%%rax), %%xmm1       \n\t"
/*		"movaps  64(%%rax), %%xmm2       \n\t"*/
/*		"movaps  96(%%rax), %%xmm3       \n\t"*/
		"movaps  16(%%rax), %%xmm4       \n\t" // load C0
		"movaps  48(%%rax), %%xmm5       \n\t"
		"movaps  80(%%rax), %%xmm6       \n\t"
		"movaps 112(%%rax), %%xmm7       \n\t"
		"                                \n\t"
		"                                \n\t"
		"addpd  %%xmm0, %%xmm9           \n\t"
		"addpd  %%xmm1, %%xmm8           \n\t"
/*		"addpd  %%xmm2, %%xmm11          \n\t"*/
/*		"addpd  %%xmm3, %%xmm10          \n\t"*/
		"addpd  %%xmm4, %%xmm13          \n\t"
		"addpd  %%xmm5, %%xmm12          \n\t"
		"addpd  %%xmm6, %%xmm15          \n\t"
		"addpd  %%xmm7, %%xmm14          \n\t"
		"                                \n\t"
		"                                \n\t"
		".DSOLVE_0:                        \n\t"
		"                                \n\t" //  9  8 11 10
		"                                \n\t" // 13 12 15 14
		"                                \n\t"
		"movq   %7, %%rax                \n\t" // load address of inv_diag_D
		"movq   %6, %%rbx                \n\t" // load address of D
		"                                \n\t"
		"movl   %12, %%ecx               \n\t" // load km
		"movl   %13, %%edx               \n\t" // load kn
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movsd   .DTHR_0(%%rip), %%xmm7    \n\t" // 1e-15
		"movsd   .DONE_0(%%rip), %%xmm6    \n\t" // 1.0
		"                                \n\t"
		"                                \n\t"
		".DA00_0:                          \n\t"
		"movsd   %%xmm9, %%xmm0          \n\t" // a_00
		"ucomisd %%xmm7, %%xmm0          \n\t"
		"jbe     .DZERO0_0                 \n\t"
		"sqrtsd  %%xmm0, %%xmm0          \n\t"
		"movsd   %%xmm6, %%xmm5          \n\t" // 1.0
		"divsd   %%xmm0, %%xmm5          \n\t" // 1.0/a_00
		"movsd   %%xmm5, (%%rax)         \n\t" // inv_diag_D[0]
		"movddup %%xmm5, %%xmm5          \n\t"
		"mulpd   %%xmm5, %%xmm9          \n\t"
		"mulpd   %%xmm5, %%xmm13         \n\t"
		".DA00_END_0:                    \n\t"
		"cmpl      $4,  %%ecx            \n\t"
		"movaps	 %%xmm9,  (%%rbx)        \n\t"
		"jge    .STORE_0_4_0               \n\t"
		"movsd	 %%xmm13, 16(%%rbx)      \n\t"
		"jmp    .STORE_0_4_END_0         \n\t"
		".STORE_0_4_0:                     \n\t"
		"movaps	 %%xmm13, 16(%%rbx)      \n\t"
		".STORE_0_4_END_0:                 \n\t"
		"                                \n\t"
		".DA11_0:                          \n\t"
		"movaps  %%xmm9, %%xmm0          \n\t"
		"shufpd  $3, %%xmm0, %%xmm0      \n\t" // a_10
		"movaps  %%xmm0,  %%xmm1         \n\t"
		"mulpd   %%xmm9, %%xmm0          \n\t"
		"mulpd   %%xmm13, %%xmm1         \n\t"
		"subpd   %%xmm0, %%xmm8          \n\t"
		"subpd   %%xmm1, %%xmm12         \n\t"
		"movaps  %%xmm8, %%xmm0          \n\t" // [a_01 a_11]
		"shufpd  $1, %%xmm0, %%xmm0      \n\t" // a_11
		"ucomisd %%xmm7, %%xmm0          \n\t"
		"jbe     .DZERO1_0                 \n\t"
		"sqrtsd  %%xmm0, %%xmm0          \n\t"
		"movsd   %%xmm6, %%xmm5          \n\t" // 1.0
		"divsd   %%xmm0, %%xmm5          \n\t" // 1.0/a_11
		"movsd   %%xmm5, 8(%%rax)        \n\t" // inv_diag_D[1]
		"movddup %%xmm5, %%xmm5          \n\t"
		"mulpd   %%xmm5, %%xmm8          \n\t"
		"mulpd   %%xmm5, %%xmm12         \n\t"
		".DA11_END_0:                    \n\t"
		"cmpl      $4,  %%ecx            \n\t"
		"movhpd	 %%xmm8,  40(%%rbx)      \n\t"
		"jge    .STORE_1_4_0               \n\t"
		"movsd	 %%xmm12, 48(%%rbx)      \n\t"
		"jmp    .STORE_1_4_END_0           \n\t"
		".STORE_1_4_0:                     \n\t"
		"movaps	 %%xmm12, 48(%%rbx)      \n\t"
		".STORE_1_4_END_0:                 \n\t"
		"                                \n\t"
		".DA22_0:                          \n\t"
		"movddup %%xmm13, %%xmm0         \n\t" // a_20
		"movddup %%xmm12, %%xmm1         \n\t" // a_21
		"mulpd   %%xmm13, %%xmm0         \n\t"
		"mulpd   %%xmm12, %%xmm1         \n\t"
		"subpd   %%xmm0, %%xmm15         \n\t"
		"subpd   %%xmm1, %%xmm15         \n\t"
		"movsd   %%xmm15, %%xmm0         \n\t" // a_22
		"ucomisd %%xmm7, %%xmm0          \n\t"
		"jbe     .DZERO2_0                 \n\t"
		"sqrtsd  %%xmm0, %%xmm0          \n\t"
		"movsd   %%xmm6, %%xmm5          \n\t" // 1.0
		"divsd   %%xmm0, %%xmm5          \n\t" // 1.0/a_11
		"movsd   %%xmm5, 16(%%rax)       \n\t" // inv_diag_D[2]
		"movddup %%xmm5, %%xmm5          \n\t"
		"mulpd   %%xmm5, %%xmm15         \n\t"
		".DA22_END_0:                    \n\t"
		"cmpl      $4,  %%ecx            \n\t"
		"jge    .STORE_2_4_0               \n\t"
		"movsd	 %%xmm15, 80(%%rbx)      \n\t"
		"jmp    .STORE_2_4_END_0           \n\t"
		".STORE_2_4_0:                     \n\t"
		"movaps	 %%xmm15, 80(%%rbx)      \n\t"
		".STORE_2_4_END_0:                 \n\t"
		"                                \n\t"
		"cmpl      $4,  %%edx            \n\t"
		"                                \n\t"
		".DA33_0:                          \n\t"
		"movaps  %%xmm13, %%xmm0         \n\t"
		"movaps  %%xmm12, %%xmm1         \n\t"
		"movaps  %%xmm15, %%xmm2         \n\t"
		"shufpd  $3, %%xmm0, %%xmm0      \n\t" // a_30
		"shufpd  $3, %%xmm1, %%xmm1      \n\t" // a_31
		"shufpd  $3, %%xmm2, %%xmm2      \n\t" // a_32
		"jl      .DEND_0                   \n\t"
		"mulpd   %%xmm13, %%xmm0         \n\t"
		"mulpd   %%xmm12, %%xmm1         \n\t"
		"mulpd   %%xmm15, %%xmm2         \n\t"
		"subpd   %%xmm0, %%xmm14         \n\t"
		"subpd   %%xmm1, %%xmm14         \n\t"
		"subpd   %%xmm2, %%xmm14         \n\t"
		"movaps  %%xmm14, %%xmm0         \n\t" // [a_23 a_33]
		"shufpd  $1, %%xmm0, %%xmm0      \n\t" // a_33
		"ucomisd %%xmm7, %%xmm0          \n\t"
		"jbe     .DZERO3_0                 \n\t"
		"sqrtsd  %%xmm0, %%xmm0          \n\t"
		"movsd   %%xmm0, 120(%%rbx)      \n\t"
		"divsd   %%xmm0, %%xmm6          \n\t" // 1.0/a_11
		"movsd   %%xmm6, 24(%%rax)       \n\t" // inv_diag_D[3]
		"                                \n\t"
		"jmp    .DEND_0                    \n\t"
		"                                \n\t"
		"                                \n\t"
		".DZERO0_0:                        \n\t"
		"xorpd  %%xmm9,  %%xmm9          \n\t"
		"movaps %%xmm9,  %%xmm13         \n\t"
		"movsd  %%xmm9,  (%%rax)         \n\t" // inv_diag_D[0]
		"jmp    .DA00_END_0              \n\t"
		"                                \n\t"
		".DZERO1_0:                        \n\t"
		"xorpd  %%xmm8,  %%xmm8          \n\t"
		"movaps %%xmm8,  %%xmm12         \n\t"
		"movsd  %%xmm8,  8(%%rax)       \n\t" // inv_diag_D[1]
		"jmp    .DA11_END_0              \n\t"
		"                                \n\t"
		".DZERO2_0:                        \n\t"
		"xorpd  %%xmm15, %%xmm15         \n\t"
		"movsd  %%xmm15, 16(%%rax)       \n\t" // inv_diag_D[2]
		"jmp    .DA22_END_0              \n\t"
		"                                \n\t"
		".DZERO3_0:                        \n\t"
		"xorpd  %%xmm14, %%xmm14         \n\t"
		"movsd  %%xmm14, 120(%%rbx)      \n\t"
		"movsd  %%xmm14, 24(%%rax)       \n\t" // inv_diag_D[3]
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"jmp    .DEND_0                    \n\t"
		".align 16                       \n\t"
		".DONE_0:                          \n\t"
		".long  0                        \n\t"
		".long  1072693248               \n\t"
		".long  0                        \n\t"
		".long  0                        \n\t"
		".align 16                       \n\t"
		".DTHR_0:                          \n\t"
		".long  2665960982               \n\t"
		".long  1020396463               \n\t"
		".long  0                        \n\t"
		".long  0                        \n\t"
		"                                \n\t"
		"                                \n\t"
		".DEND_0:                          \n\t"
		"                                \n\t"
		: // output operands (none)
		: // input operands
		  "m" (ki_add),		// %0
		  "m" (kl_add),		// %1
		  "m" (ki_sub),		// %2
		  "m" (Ap),			// %3
		  "m" (Bp),			// %4
		  "m" (C),			// %5
		  "m" (D),			// %6
		  "m" (inv_diag_D),	// %7
		  "m" (alg),		// %8
		  "m" (Am),			// %9
		  "m" (Bm),			// %10
		  "m" (tri_A),		// %11
		  "m" (km),			// %12
		  "m" (kn)			// %13
		: // register clobber list
		  "rax", "rbx", "rcx", "rdx", "rsi", //"rdx", //"rdi", "r8", "r9", "r10", "r11",
		  "xmm0", "xmm1", "xmm2", "xmm3",
		  "xmm4", "xmm5", "xmm6", "xmm7",
		  "xmm8", "xmm9", "xmm10", "xmm11",
		  "xmm12", "xmm13", "xmm14", "xmm15",
		  "memory"
	);
}



void kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(int km, int kn, int kadd, int tri_A, double *Ap, double *Bp, int ksub, double *Am, double *Bm, int alg, double *C, double *D, double *inv_diag_D)
	{

	const int bs = 4;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1,
		c_00=0, 
		c_10=0, c_11=0, 
		c_20=0, c_21=0,
		c_30=0, c_31=0;
		
	k = 0;

	if(kadd>0)
		{

		if(tri_A==1)
			{

			// initial triangle

			if(kadd>=4)
				{

				// k = 0
				a_0 = Ap[0+bs*0];

				b_0 = Bp[0+bs*0];
				
				c_00 += a_0 * b_0;


				// k = 1
				a_0 = Ap[0+bs*1];
				a_1 = Ap[1+bs*1];
				
				b_0 = Bp[0+bs*1];
				b_1 = Bp[1+bs*1];
				
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;

				c_11 += a_1 * b_1;


				// k = 2
				a_0 = Ap[0+bs*2];
				a_1 = Ap[1+bs*2];
				a_2 = Ap[2+bs*2];
				
				b_0 = Bp[0+bs*2];
				b_1 = Bp[1+bs*2];
				
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;
				c_20 += a_2 * b_0;

				c_11 += a_1 * b_1;
				c_21 += a_2 * b_1;


				// k = 3
				a_0 = Ap[0+bs*3];
				a_1 = Ap[1+bs*3];
				a_2 = Ap[2+bs*3];
				a_3 = Ap[3+bs*3];
					
				b_0 = Bp[0+bs*3];
				b_1 = Bp[1+bs*3];
					
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;
				c_20 += a_2 * b_0;
				c_30 += a_3 * b_0;

				c_11 += a_1 * b_1;
				c_21 += a_2 * b_1;
				c_31 += a_3 * b_1;

				Ap += 16;
				Bp += 16;
				k  += 4;

				}
			else
				{

				// k = 0
				a_0 = Ap[0+bs*0];

				b_0 = Bp[0+bs*0];
				
				c_00 += a_0 * b_0;

				k  += 1;

				if(kadd>1)
					{

					// k = 1
					a_0 = Ap[0+bs*1];
					a_1 = Ap[1+bs*1];
					
					b_0 = Bp[0+bs*1];
					b_1 = Bp[1+bs*1];
					
					c_00 += a_0 * b_0;
					c_10 += a_1 * b_0;

					c_11 += a_1 * b_1;

					k  += 1;

					if(kadd>2)
						{

						// k = 2
						a_0 = Ap[0+bs*2];
						a_1 = Ap[1+bs*2];
						a_2 = Ap[2+bs*2];
						
						b_0 = Bp[0+bs*2];
						b_1 = Bp[1+bs*2];
						
						c_00 += a_0 * b_0;
						c_10 += a_1 * b_0;
						c_20 += a_2 * b_0;

						c_11 += a_1 * b_1;
						c_21 += a_2 * b_1;

						k  += 1;

						}

					}

				}

			}

		for(; k<kadd-3; k+=4)
			{
			
			a_0 = Ap[0+bs*0];
			a_1 = Ap[1+bs*0];
			a_2 = Ap[2+bs*0];
			a_3 = Ap[3+bs*0];
			
			b_0 = Bp[0+bs*0];
			b_1 = Bp[1+bs*0];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;
			c_20 += a_2 * b_0;
			c_30 += a_3 * b_0;

			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;


			a_0 = Ap[0+bs*1];
			a_1 = Ap[1+bs*1];
			a_2 = Ap[2+bs*1];
			a_3 = Ap[3+bs*1];
			
			b_0 = Bp[0+bs*1];
			b_1 = Bp[1+bs*1];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;
			c_20 += a_2 * b_0;
			c_30 += a_3 * b_0;

			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;


			a_0 = Ap[0+bs*2];
			a_1 = Ap[1+bs*2];
			a_2 = Ap[2+bs*2];
			a_3 = Ap[3+bs*2];
			
			b_0 = Bp[0+bs*2];
			b_1 = Bp[1+bs*2];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;
			c_20 += a_2 * b_0;
			c_30 += a_3 * b_0;

			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;


			a_0 = Ap[0+bs*3];
			a_1 = Ap[1+bs*3];
			a_2 = Ap[2+bs*3];
			a_3 = Ap[3+bs*3];
			
			b_0 = Bp[0+bs*3];
			b_1 = Bp[1+bs*3];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;
			c_20 += a_2 * b_0;
			c_30 += a_3 * b_0;

			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;
			
			
			Ap += 16;
			Bp += 16;

			}
		for(; k<kadd; k++)
			{
			
			a_0 = Ap[0+bs*0];
			a_1 = Ap[1+bs*0];
			a_2 = Ap[2+bs*0];
			a_3 = Ap[3+bs*0];
			
			b_0 = Bp[0+bs*0];
			b_1 = Bp[1+bs*0];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;
			c_20 += a_2 * b_0;
			c_30 += a_3 * b_0;

			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;


			Ap += 4;
			Bp += 4;

			}

		}

	for(k=0; k<ksub-3; k+=4)
		{
		
		a_0 = Am[0+bs*0];
		a_1 = Am[1+bs*0];
		a_2 = Am[2+bs*0];
		a_3 = Am[3+bs*0];
		
		b_0 = Bm[0+bs*0];
		b_1 = Bm[1+bs*0];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;


		a_0 = Am[0+bs*1];
		a_1 = Am[1+bs*1];
		a_2 = Am[2+bs*1];
		a_3 = Am[3+bs*1];
		
		b_0 = Bm[0+bs*1];
		b_1 = Bm[1+bs*1];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;


		a_0 = Am[0+bs*2];
		a_1 = Am[1+bs*2];
		a_2 = Am[2+bs*2];
		a_3 = Am[3+bs*2];
		
		b_0 = Bm[0+bs*2];
		b_1 = Bm[1+bs*2];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;


		a_0 = Am[0+bs*3];
		a_1 = Am[1+bs*3];
		a_2 = Am[2+bs*3];
		a_3 = Am[3+bs*3];
		
		b_0 = Bm[0+bs*3];
		b_1 = Bm[1+bs*3];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;
		
		
		Am += 16;
		Bm += 16;

		}

	if(alg!=0)
		{
		c_00 += C[0+bs*0];
		c_10 += C[1+bs*0];
		c_20 += C[2+bs*0];
		c_30 += C[3+bs*0];

		c_11 += C[1+bs*1];
		c_21 += C[2+bs*1];
		c_31 += C[3+bs*1];
		}
	
	// first column
	if(c_00 > 1e-15)
		{
		c_00 = sqrt(c_00);
		D[0+bs*0] = c_00;
		c_00 = 1.0/c_00;
		inv_diag_D[0] = c_00;
		c_10 *= c_00;
		c_20 *= c_00;
		c_30 *= c_00;
		}
	else
		{
		c_00 = 0.0;
		D[0+bs*0] = c_00;
		inv_diag_D[0] = c_00;
		c_10 = 0.0;
		c_20 = 0.0;
		c_30 = 0.0;
		}
	D[1+bs*0] = c_10;
	D[2+bs*0] = c_20;
	if(km>=4)
		{
		D[3+bs*0] = c_30;
		}
		
	if(kn==1)
		return;

	// second column
	c_11 -= c_10*c_10;
	c_21 -= c_20*c_10;
	c_31 -= c_30*c_10;
	if(c_11 > 1e-15)
		{
		c_11 = sqrt(c_11);
		D[1+bs*1] = c_11;
		c_11 = 1.0/c_11;
		inv_diag_D[1] = c_11;
		c_21 *= c_11;
		c_31 *= c_11;
		}
	else
		{
		c_11 = 0.0;
		D[1+bs*1] = c_11;
		inv_diag_D[1] = c_11;
		c_21 = 0.0;
		c_31 = 0.0;
		}
	D[2+bs*1] = c_21;
	if(km>=4)
		{
		D[3+bs*1] = c_31;
		}

	}


void kernel_dsyrk_dpotrf_nt_2x2_vs_lib4_new(int km, int kn, int kadd, int tri_A, double *Ap, double *Bp, int ksub, double *Am, double *Bm, int alg, double *C, double *D, double *inv_diag_D)
	{

	const int bs = 4;

	int k;

	double
		a_0, a_1,
		b_0, b_1,
		c_00=0, 
		c_10=0, c_11=0; 
		
	k = 0;

	if(kadd>0)
		{

		if(tri_A==1)
			{

			// initial triangle

			if(kadd>=4)
				{

				// k = 0
				a_0 = Ap[0+bs*0];

				b_0 = Bp[0+bs*0];
				
				c_00 += a_0 * b_0;


				// k = 1
				a_0 = Ap[0+bs*1];
				a_1 = Ap[1+bs*1];
				
				b_0 = Bp[0+bs*1];
				b_1 = Bp[1+bs*1];
				
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;

				c_11 += a_1 * b_1;


				// k = 2
				a_0 = Ap[0+bs*2];
				a_1 = Ap[1+bs*2];
				
				b_0 = Bp[0+bs*2];
				b_1 = Bp[1+bs*2];
				
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;

				c_11 += a_1 * b_1;


				// k = 3
				a_0 = Ap[0+bs*3];
				a_1 = Ap[1+bs*3];
					
				b_0 = Bp[0+bs*3];
				b_1 = Bp[1+bs*3];
					
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;

				c_11 += a_1 * b_1;

				Ap += 16;
				Bp += 16;
				k  += 4;

				}
			else
				{

				// k = 0
				a_0 = Ap[0+bs*0];

				b_0 = Bp[0+bs*0];
				
				c_00 += a_0 * b_0;

				k  += 1;

				if(kadd>1)
					{

					// k = 1
					a_0 = Ap[0+bs*1];
					a_1 = Ap[1+bs*1];
					
					b_0 = Bp[0+bs*1];
					b_1 = Bp[1+bs*1];
					
					c_00 += a_0 * b_0;
					c_10 += a_1 * b_0;

					c_11 += a_1 * b_1;

					k  += 1;

					if(kadd>2)
						{

						// k = 2
						a_0 = Ap[0+bs*2];
						a_1 = Ap[1+bs*2];
						
						b_0 = Bp[0+bs*2];
						b_1 = Bp[1+bs*2];
						
						c_00 += a_0 * b_0;
						c_10 += a_1 * b_0;

						c_11 += a_1 * b_1;

						k  += 1;

						}

					}

				}

			}

		for(; k<kadd-3; k+=4)
			{
			
			a_0 = Ap[0+bs*0];
			a_1 = Ap[1+bs*0];
			
			b_0 = Bp[0+bs*0];
			b_1 = Bp[1+bs*0];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_11 += a_1 * b_1;


			a_0 = Ap[0+bs*1];
			a_1 = Ap[1+bs*1];
			
			b_0 = Bp[0+bs*1];
			b_1 = Bp[1+bs*1];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_11 += a_1 * b_1;


			a_0 = Ap[0+bs*2];
			a_1 = Ap[1+bs*2];
			
			b_0 = Bp[0+bs*2];
			b_1 = Bp[1+bs*2];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_11 += a_1 * b_1;


			a_0 = Ap[0+bs*3];
			a_1 = Ap[1+bs*3];
			
			b_0 = Bp[0+bs*3];
			b_1 = Bp[1+bs*3];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_11 += a_1 * b_1;
			
			
			Ap += 16;
			Bp += 16;

			}
		for(; k<kadd; k++)
			{
			
			a_0 = Ap[0+bs*0];
			a_1 = Ap[1+bs*0];
			
			b_0 = Bp[0+bs*0];
			b_1 = Bp[1+bs*0];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_11 += a_1 * b_1;


			Ap += 4;
			Bp += 4;

			}

		}

	for(k=0; k<ksub-3; k+=4)
		{
		
		a_0 = Am[0+bs*0];
		a_1 = Am[1+bs*0];
		
		b_0 = Bm[0+bs*0];
		b_1 = Bm[1+bs*0];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_11 -= a_1 * b_1;


		a_0 = Am[0+bs*1];
		a_1 = Am[1+bs*1];
		
		b_0 = Bm[0+bs*1];
		b_1 = Bm[1+bs*1];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_11 -= a_1 * b_1;


		a_0 = Am[0+bs*2];
		a_1 = Am[1+bs*2];
		
		b_0 = Bm[0+bs*2];
		b_1 = Bm[1+bs*2];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_11 -= a_1 * b_1;


		a_0 = Am[0+bs*3];
		a_1 = Am[1+bs*3];
		
		b_0 = Bm[0+bs*3];
		b_1 = Bm[1+bs*3];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_11 -= a_1 * b_1;
		
		
		Am += 16;
		Bm += 16;

		}

	if(alg!=0)
		{
		c_00 += C[0+bs*0];
		c_10 += C[1+bs*0];

		c_11 += C[1+bs*1];
		}
	
	// first column
	if(c_00 > 1e-15)
		{
		c_00 = sqrt(c_00);
		D[0+bs*0] = c_00;
		c_00 = 1.0/c_00;
		inv_diag_D[0] = c_00;
		c_10 *= c_00;
		}
	else
		{
		c_00 = 0.0;
		D[0+bs*0] = c_00;
		inv_diag_D[0] = c_00;
		c_10 = 0.0;
		}
	if(km>=2)
		{
		D[1+bs*0] = c_10;
		}
		
	if(kn==1 || km==1)
		return;

	// second column
	c_11 -= c_10*c_10;
	if(c_11 > 1e-15)
		{
		c_11 = sqrt(c_11);
		D[1+bs*1] = c_11;
		c_11 = 1.0/c_11;
		inv_diag_D[1] = c_11;
		}
	else
		{
		c_11 = 0.0;
		D[1+bs*1] = c_11;
		inv_diag_D[1] = c_11;
		}

	}


// old kernels

// normal-transposed, 4x4 with data packed in 4
void kernel_dsyrk_dpotrf_nt_4x4_vs_lib4(int km, int kn, int tri, int kadd, int ksub, double *Ap, double *Bp, double *Am, double *Bm, double *C, double *D, double *fact, int alg, int fast_rsqrt)
	{

	int ki_add = kadd/4;
	int kl_add = kadd%4;
	int ki_sub = ksub/4;


	__asm__ volatile
	(
		"                                \n\t"
		"                                \n\t"
		"movq          %3, %%rax         \n\t" // load address of A
		"movq          %4, %%rbx         \n\t" // load address of B
		"                                \n\t"
		"                                \n\t"
		"xorpd     %%xmm3,  %%xmm3       \n\t" // zero registers
		"movaps    %%xmm3,  %%xmm4       \n\t"
		"movaps    %%xmm3,  %%xmm5       \n\t"
		"movaps    %%xmm3,  %%xmm6       \n\t"
		"movaps    %%xmm3,  %%xmm7       \n\t"
		"movaps    %%xmm3,  %%xmm8       \n\t"
		"movaps    %%xmm3,  %%xmm9       \n\t"
/*		"movaps    %%xmm3, %%xmm10       \n\t"*/
/*		"movaps    %%xmm3, %%xmm11       \n\t"*/
		"movaps    %%xmm3, %%xmm12       \n\t"
		"movaps    %%xmm3, %%xmm13       \n\t"
		"movaps    %%xmm3, %%xmm14       \n\t"
		"movaps    %%xmm3, %%xmm15       \n\t"
		"                                \n\t"
		"                                \n\t"
		"movl      %0, %%esi             \n\t" // ki_add
		"movl      %1, %%ecx             \n\t" // kl_add
		"addl      %%esi, %%ecx          \n\t" // ki_add + kl_add
		"testl  %%ecx, %%ecx             \n\t" 
		"je     .DCONSIDERSUB            \n\t" // kadd = 0
		"                                \n\t"
		"                                \n\t"
		"movaps        0(%%rax), %%xmm0  \n\t" // initialize loop by pre-loading elements
		"movaps       16(%%rax), %%xmm1  \n\t" // of a and b.
		"movaps        0(%%rbx), %%xmm2  \n\t"
		"                                \n\t"
		"                                \n\t"
		"movl      %11, %%ecx            \n\t"
		"cmpl      $1,  %%ecx            \n\t"
		"jne       .DCONSIDERLOOPADD     \n\t"
		"                                \n\t"
		"                                \n\t"
		"testl  %%esi, %%esi             \n\t" // logical AND
		"je     .DTRIADD                 \n\t" // if ki_add == 0, jump
		"                                \n\t"
		"                                \n\t"
//		"movaps       16(%%rbx), %%xmm6  \n\t"
//		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulsd   %%xmm0, %%xmm2          \n\t"
		"                                \n\t"
//		"mulsd   %%xmm0, %%xmm7          \n\t"
		"                                \n\t"
		"addsd   %%xmm2, %%xmm8          \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
//		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
//		"mulsd   %%xmm0, %%xmm6          \n\t"
		"                                \n\t"
//		"addsd   %%xmm7, %%xmm9          \n\t"
//		"mulsd   %%xmm0, %%xmm4          \n\t"
		"movaps       32(%%rax), %%xmm0  \n\t"
//		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
//		"addsd   %%xmm6, %%xmm10         \n\t" // iteration 1
//		"movaps       48(%%rbx), %%xmm6  \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"                                \n\t"
//		"addsd   %%xmm4, %%xmm11         \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       64(%%rbx), %%xmm2  \n\t"
//		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
//		"mulpd   %%xmm0, %%xmm6          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
//		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       64(%%rax), %%xmm0  \n\t"
		"movaps       80(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
//		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 2
		"movaps       80(%%rbx), %%xmm6  \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulsd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
//		"addpd   %%xmm4, %%xmm11         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulsd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       96(%%rbx), %%xmm2  \n\t"
		"addsd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
//		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulsd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addsd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
//		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps       96(%%rax), %%xmm0  \n\t"
		"mulsd   %%xmm1, %%xmm5          \n\t"
		"movaps      112(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
//		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 3
		"movaps      112(%%rbx), %%xmm6  \n\t"
		"addsd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"addq      $128, %%rax           \n\t" // A0 += 16
		"                                \n\t"
//		"addpd   %%xmm4, %%xmm11         \n\t"
		"addsd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"addq      $128, %%rbx           \n\t" // B += 16
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps         (%%rbx), %%xmm2  \n\t"
		"addpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
//		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"decl    %%esi                   \n\t" // i -= 1;
		"addpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
//		"mulpd   %%xmm0, %%xmm4          \n\t"
		"movaps         (%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       16(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"jmp     .DCONSIDERLOOPADD       \n\t"
		"                                \n\t"
		"                                \n\t"
		".DTRIADD:                       \n\t"
		"                                \n\t"
		"movl    %1, %%esi               \n\t"
		"                                \n\t"
		"                                \n\t"
//		"movaps       16(%%rbx), %%xmm6  \n\t" // iteration 0
//		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulsd   %%xmm0, %%xmm2          \n\t"
		"addsd   %%xmm2, %%xmm8          \n\t"
		"                                \n\t"
//		"mulsd   %%xmm0, %%xmm7          \n\t"
//		"addsd   %%xmm7, %%xmm9          \n\t"
		"                                \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
//		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
//		"mulsd   %%xmm0, %%xmm6          \n\t"
//		"addsd   %%xmm6, %%xmm10         \n\t"
		"                                \n\t"
//		"mulsd   %%xmm0, %%xmm4          \n\t"
//		"addsd   %%xmm4, %%xmm11         \n\t"
		"movaps       32(%%rax), %%xmm0  \n\t"
//		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
//		"cmpl    $1, %%esi               \n\t"
//		"jle    .DCONSIDERSUB_NOCLEAN    \n\t" //
		"cmpl    $2, %%esi               \n\t"
		"jl     .DCONSIDERSUB_NOCLEAN    \n\t" //
		"                                \n\t"
		"                                \n\t"
//		"movaps       48(%%rbx), %%xmm6  \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"                                \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"                                \n\t"
		"movaps       64(%%rbx), %%xmm2  \n\t"
//		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
//		"mulpd   %%xmm0, %%xmm6          \n\t"
//		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 1
		"                                \n\t"
//		"mulpd   %%xmm0, %%xmm4          \n\t"
//		"addpd   %%xmm4, %%xmm11         \n\t"
		"movaps       64(%%rax), %%xmm0  \n\t"
		"movaps       80(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
//		"cmpl    $2, %%esi               \n\t"
//		"jle    .DCONSIDERSUB_NOCLEAN    \n\t" //
		"je     .DCONSIDERSUB_NOCLEAN    \n\t" //
		"                                \n\t"
		"                                \n\t"
		"movaps       80(%%rbx), %%xmm6  \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulsd   %%xmm1, %%xmm3          \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"addsd   %%xmm3, %%xmm12         \n\t"
		"                                \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulsd   %%xmm1, %%xmm5          \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addsd   %%xmm5, %%xmm13         \n\t"
		"                                \n\t"
//		"movaps       96(%%rbx), %%xmm2  \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
//		"mulpd   %%xmm0, %%xmm6          \n\t"
		"mulsd   %%xmm1, %%xmm3          \n\t"
//		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 2
		"addsd   %%xmm3, %%xmm14         \n\t"
		"                                \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
//		"mulpd   %%xmm0, %%xmm4          \n\t"
//		"movaps       96(%%rax), %%xmm0  \n\t"
		"mulsd   %%xmm1, %%xmm5          \n\t"
//		"movaps      112(%%rax), %%xmm1  \n\t"
//		"addpd   %%xmm4, %%xmm11         \n\t"
		"addsd   %%xmm5, %%xmm15         \n\t"
		"                                \n\t"
		"                                \n\t"
		"jmp    .DCONSIDERSUB_NOCLEAN    \n\t" //
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERLOOPADD:              \n\t" // MAIN LOOP add
		"                                \n\t"
		"                                \n\t"
		"testl  %%esi, %%esi             \n\t" // logical AND
		"je     .DCONSIDERADD            \n\t" // if ki_add == 0, jump
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DLOOPADD:                      \n\t" // MAIN LOOP add
		"                                \n\t"
/*		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 0*/
		"movaps       16(%%rbx), %%xmm6  \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
/*		"addpd   %%xmm4, %%xmm11         \n\t"*/
		"addpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
		"addpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps       32(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 1*/
		"movaps       48(%%rbx), %%xmm6  \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
/*		"addpd   %%xmm4, %%xmm11         \n\t"*/
		"addpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       64(%%rbx), %%xmm2  \n\t"
		"addpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps       64(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       80(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 2*/
		"movaps       80(%%rbx), %%xmm6  \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
/*		"addpd   %%xmm4, %%xmm11         \n\t"*/
		"addpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       96(%%rbx), %%xmm2  \n\t"
		"addpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps       96(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps      112(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 3*/
		"movaps      112(%%rbx), %%xmm6  \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addq      $128, %%rax           \n\t" // A0 += 16
		"                                \n\t"
/*		"addpd   %%xmm4, %%xmm11         \n\t"*/
		"addpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addq      $128, %%rbx           \n\t" // B += 16
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps         (%%rbx), %%xmm2  \n\t"
		"addpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"decl    %%esi                   \n\t" // i -= 1;
		"addpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps         (%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       16(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"jne    .DLOOPADD                \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERADD:                  \n\t"
		"                                \n\t"
		"movl      %1, %%esi             \n\t" // i = k_left;
		"testl  %%esi, %%esi             \n\t" // check i via logical AND.
		"je     .DCONSIDERSUB            \n\t" // if i == 0, we're done; jump to end.
		"                                \n\t" // else, we prepare to enter k_left loop.
		"                                \n\t"
		"                                \n\t"
		".DLOOPLEFT:                     \n\t" // EDGE LOOP
		"                                \n\t"
/*		"addpd   %%xmm6, %%xmm10         \n\t" // iteration 0*/
		"movaps       16(%%rbx), %%xmm6  \n\t"
		"addpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
/*		"addpd   %%xmm4, %%xmm11         \n\t"*/
		"addpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addpd   %%xmm2, %%xmm8          \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
		"addpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addpd   %%xmm7, %%xmm9          \n\t"
		"addpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps       32(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"addq          $32, %%rax        \n\t" // A += 4
		"addq          $32, %%rbx        \n\t" // B += 4
		"                                \n\t"
		"                                \n\t"
		"decl    %%esi                   \n\t" // i -= 1;
		"jne    .DLOOPLEFT               \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERSUB:                  \n\t" // padd
		"                                \n\t"
		"                                \n\t"
/*		"addpd   %%xmm6, %%xmm10         \n\t"*/
		"addpd   %%xmm3, %%xmm14         \n\t"
/*		"addpd   %%xmm4, %%xmm11         \n\t"*/
		"addpd   %%xmm5, %%xmm15         \n\t"
		"                                \n\t"
		"                                \n\t"
		".DCONSIDERSUB_NOCLEAN:          \n\t" // padd
		"                                \n\t"
		"                                \n\t"
		"xorpd     %%xmm3,  %%xmm3       \n\t"
		"movaps    %%xmm3,  %%xmm4       \n\t"
		"movaps    %%xmm3,  %%xmm5       \n\t"
		"movaps    %%xmm3,  %%xmm6       \n\t"
		"                                \n\t"
		"                                \n\t"
		"movl   %2, %%esi                \n\t"
		"testl  %%esi, %%esi             \n\t" // check i via logical AND.
		"je     .DPOSTACC                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movq   %9,  %%rax               \n\t"
		"movq   %10, %%rbx               \n\t"
		"                                \n\t"
		"movaps        0(%%rax), %%xmm0  \n\t" // initialize loop by pre-loading elements
		"movaps       16(%%rax), %%xmm1  \n\t" // of a and b.
		"movaps        0(%%rbx), %%xmm2  \n\t"
		"                                \n\t"
		"                                \n\t"
		".DLOOPSUB:                      \n\t" // main loop 2
		"                                \n\t"
/*		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 0*/
		"movaps       16(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
/*		"subpd   %%xmm4, %%xmm11         \n\t"*/
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps       32(%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps       32(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       48(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 1*/
		"movaps       48(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
/*		"subpd   %%xmm4, %%xmm11         \n\t"*/
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps       64(%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps       64(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       80(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 2*/
		"movaps       80(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
/*		"subpd   %%xmm4, %%xmm11         \n\t"*/
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps       96(%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps       96(%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps      112(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
/*		"subpd   %%xmm6, %%xmm10         \n\t" // iteration 3*/
		"movaps      112(%%rbx), %%xmm6  \n\t"
		"subpd   %%xmm3, %%xmm14         \n\t"
		"movaps  %%xmm2, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm2, %%xmm7  \n\t"
		"mulpd   %%xmm0, %%xmm2          \n\t"
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"addq      $128, %%rax           \n\t" // A0 += 16
		"                                \n\t"
/*		"subpd   %%xmm4, %%xmm11         \n\t"*/
		"subpd   %%xmm5, %%xmm15         \n\t"
		"movaps  %%xmm7, %%xmm5          \n\t"
		"mulpd   %%xmm0, %%xmm7          \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"                                \n\t"
		"addq      $128, %%rbx           \n\t" // B += 16
		"                                \n\t"
		"subpd   %%xmm2, %%xmm8          \n\t"
		"movaps         (%%rbx), %%xmm2  \n\t"
		"subpd   %%xmm3, %%xmm12         \n\t"
		"movaps  %%xmm6, %%xmm3          \n\t"
		"pshufd   $0x4e, %%xmm6, %%xmm4  \n\t"
/*		"mulpd   %%xmm0, %%xmm6          \n\t"*/
		"mulpd   %%xmm1, %%xmm3          \n\t"
		"                                \n\t"
		"                                \n\t"
		"subpd   %%xmm7, %%xmm9          \n\t"
		"decl    %%esi                   \n\t" // i -= 1;
		"subpd   %%xmm5, %%xmm13         \n\t"
		"movaps  %%xmm4, %%xmm5          \n\t"
/*		"mulpd   %%xmm0, %%xmm4          \n\t"*/
		"movaps         (%%rax), %%xmm0  \n\t"
		"mulpd   %%xmm1, %%xmm5          \n\t"
		"movaps       16(%%rax), %%xmm1  \n\t"
		"                                \n\t"
		"                                \n\t"
		"jne    .DLOOPSUB                \n\t" // iterate again if i != 0.
		"                                \n\t"
		"                                \n\t"
/*		"subpd   %%xmm6, %%xmm10         \n\t"*/
		"subpd   %%xmm3, %%xmm14         \n\t"
/*		"subpd   %%xmm4, %%xmm11         \n\t"*/
		"subpd   %%xmm5, %%xmm15         \n\t"
		"                                \n\t"
		"                                \n\t"
		".DPOSTACC:                      \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movaps   %%xmm8,  %%xmm0        \n\t"
		"movsd    %%xmm9,  %%xmm8        \n\t"
		"movsd    %%xmm0,  %%xmm9        \n\t"
		"                                \n\t"
/*		"movaps  %%xmm10,  %%xmm0        \n\t"*/
/*		"movsd   %%xmm11, %%xmm10        \n\t"*/
/*		"movsd    %%xmm0, %%xmm11        \n\t"*/
		"                                \n\t"
		"movaps  %%xmm12,  %%xmm0        \n\t"
		"movsd   %%xmm13, %%xmm12        \n\t"
		"movsd    %%xmm0, %%xmm13        \n\t"
		"                                \n\t"
		"movaps  %%xmm14,  %%xmm0        \n\t"
		"movsd   %%xmm15, %%xmm14        \n\t"
		"movsd    %%xmm0, %%xmm15        \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movl      %8, %%esi             \n\t" // alg
		"testl  %%esi, %%esi             \n\t" // check alg via logical AND.
		"je     .DSOLVE                  \n\t" // if alg != 0, jump 
		"                                \n\t"
		"                                \n\t"
		"movq   %5, %%rax                \n\t" // load address of C
		"                                \n\t"
		"                                \n\t"
		"movaps  (%%rax),   %%xmm0       \n\t" // load C0
		"movaps  32(%%rax), %%xmm1       \n\t"
/*		"movaps  64(%%rax), %%xmm2       \n\t"*/
/*		"movaps  96(%%rax), %%xmm3       \n\t"*/
		"movaps  16(%%rax), %%xmm4       \n\t" // load C0
		"movaps  48(%%rax), %%xmm5       \n\t"
		"movaps  80(%%rax), %%xmm6       \n\t"
		"movaps 112(%%rax), %%xmm7       \n\t"
		"                                \n\t"
		"                                \n\t"
		"addpd  %%xmm0, %%xmm9           \n\t"
		"addpd  %%xmm1, %%xmm8           \n\t"
/*		"addpd  %%xmm2, %%xmm11          \n\t"*/
/*		"addpd  %%xmm3, %%xmm10          \n\t"*/
		"addpd  %%xmm4, %%xmm13          \n\t"
		"addpd  %%xmm5, %%xmm12          \n\t"
		"addpd  %%xmm6, %%xmm15          \n\t"
		"addpd  %%xmm7, %%xmm14          \n\t"
		"                                \n\t"
		"                                \n\t"
		".DSOLVE:                        \n\t"
		"                                \n\t" //  9  8 11 10
		"                                \n\t" // 13 12 15 14
		"                                \n\t"
		"movq   %7, %%rax                \n\t" // load address of fact
		"movq   %6, %%rbx                \n\t" // load address of D
		"                                \n\t"
		"movl   %12, %%ecx               \n\t" // load km
		"movl   %13, %%edx               \n\t" // load kn
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"movsd   .DTHR(%%rip), %%xmm7    \n\t" // 1e-15
		"movsd   .DONE(%%rip), %%xmm6    \n\t" // 1.0
		"                                \n\t"
		"                                \n\t"
		".DA00:                          \n\t"
		"movsd   %%xmm9, %%xmm0          \n\t" // a_00
		"ucomisd %%xmm7, %%xmm0          \n\t"
		"jbe     .DZERO0                 \n\t"
		"sqrtsd  %%xmm0, %%xmm0          \n\t"
		"movsd   %%xmm6, %%xmm5          \n\t" // 1.0
		"divsd   %%xmm0, %%xmm5          \n\t" // 1.0/a_00
		"cmpl      $4,  %%ecx            \n\t"
		"movsd   %%xmm5, (%%rax)         \n\t" // fact[0]
		"movddup %%xmm5, %%xmm5          \n\t"
		"mulpd   %%xmm5, %%xmm9          \n\t"
		"mulpd   %%xmm5, %%xmm13         \n\t"
		"movaps	 %%xmm9,  (%%rbx)        \n\t"
		"jge    .STORE_0_4               \n\t"
		"movsd	 %%xmm13, 16(%%rbx)      \n\t"
		"jmp    .STORE_0_4_END           \n\t"
		".STORE_0_4:                     \n\t"
		"movaps	 %%xmm13, 16(%%rbx)      \n\t"
		".STORE_0_4_END:                 \n\t"
		"                                \n\t"
		".DA11:                          \n\t"
		"movaps  %%xmm9, %%xmm0          \n\t"
		"shufpd  $3, %%xmm0, %%xmm0      \n\t" // a_10
		"movsd   %%xmm0, 8(%%rax)        \n\t" // fact[1]
		"movaps  %%xmm0,  %%xmm1         \n\t"
		"mulpd   %%xmm9, %%xmm0          \n\t"
		"mulpd   %%xmm13, %%xmm1         \n\t"
		"subpd   %%xmm0, %%xmm8          \n\t"
		"subpd   %%xmm1, %%xmm12         \n\t"
		"movaps  %%xmm8, %%xmm0          \n\t" // [a_01 a_11]
		"shufpd  $1, %%xmm0, %%xmm0      \n\t" // a_11
		"ucomisd %%xmm7, %%xmm0          \n\t"
		"jbe     .DZERO1                 \n\t"
		"sqrtsd  %%xmm0, %%xmm0          \n\t"
		"movsd   %%xmm6, %%xmm5          \n\t" // 1.0
		"divsd   %%xmm0, %%xmm5          \n\t" // 1.0/a_11
		"cmpl      $4,  %%ecx            \n\t"
		"movsd   %%xmm5, 16(%%rax)       \n\t" // fact[2]
		"movddup %%xmm5, %%xmm5          \n\t"
		"mulpd   %%xmm5, %%xmm8          \n\t"
		"mulpd   %%xmm5, %%xmm12         \n\t"
		"movhpd	 %%xmm8,  40(%%rbx)      \n\t"
		"jge    .STORE_1_4               \n\t"
		"movsd	 %%xmm12, 48(%%rbx)      \n\t"
		"jmp    .STORE_1_4_END           \n\t"
		".STORE_1_4:                     \n\t"
		"movaps	 %%xmm12, 48(%%rbx)      \n\t"
		".STORE_1_4_END:                 \n\t"
		"                                \n\t"
		".DA22:                          \n\t"
		"movddup %%xmm13, %%xmm0         \n\t" // a_20
		"movddup %%xmm12, %%xmm1         \n\t" // a_21
		"movsd   %%xmm0, 24(%%rax)       \n\t" // fact[3]
		"movsd   %%xmm1, 32(%%rax)       \n\t" // fact[4]
		"mulpd   %%xmm13, %%xmm0         \n\t"
		"mulpd   %%xmm12, %%xmm1         \n\t"
		"subpd   %%xmm0, %%xmm15         \n\t"
		"subpd   %%xmm1, %%xmm15         \n\t"
		"movsd   %%xmm15, %%xmm0         \n\t" // a_22
		"ucomisd %%xmm7, %%xmm0          \n\t"
		"jbe     .DZERO2                 \n\t"
		"sqrtsd  %%xmm0, %%xmm0          \n\t"
		"movsd   %%xmm6, %%xmm5          \n\t" // 1.0
		"divsd   %%xmm0, %%xmm5          \n\t" // 1.0/a_11
		"cmpl      $4,  %%ecx            \n\t"
		"movsd   %%xmm5, 40(%%rax)       \n\t" // fact[5]
		"movddup %%xmm5, %%xmm5          \n\t"
		"mulpd   %%xmm5, %%xmm15         \n\t"
		"jge    .STORE_2_4               \n\t"
		"movsd	 %%xmm15, 80(%%rbx)      \n\t"
		"jmp    .STORE_2_4_END           \n\t"
		".STORE_2_4:                     \n\t"
		"movaps	 %%xmm15, 80(%%rbx)      \n\t"
		".STORE_2_4_END:                 \n\t"
		"                                \n\t"
		"cmpl      $4,  %%edx            \n\t"
		"                                \n\t"
		".DA33:                          \n\t"
		"movaps  %%xmm13, %%xmm0         \n\t"
		"movaps  %%xmm12, %%xmm1         \n\t"
		"movaps  %%xmm15, %%xmm2         \n\t"
		"shufpd  $3, %%xmm0, %%xmm0      \n\t" // a_30
		"shufpd  $3, %%xmm1, %%xmm1      \n\t" // a_31
		"shufpd  $3, %%xmm2, %%xmm2      \n\t" // a_32
		"movsd   %%xmm0, 48(%%rax)       \n\t" // fact[6]
		"movsd   %%xmm1, 56(%%rax)       \n\t" // fact[7]
		"movsd   %%xmm2, 64(%%rax)       \n\t" // fact[8]
		"jl      .DEND                   \n\t"
		"mulpd   %%xmm13, %%xmm0         \n\t"
		"mulpd   %%xmm12, %%xmm1         \n\t"
		"mulpd   %%xmm15, %%xmm2         \n\t"
		"subpd   %%xmm0, %%xmm14         \n\t"
		"subpd   %%xmm1, %%xmm14         \n\t"
		"subpd   %%xmm2, %%xmm14         \n\t"
		"movaps  %%xmm14, %%xmm0         \n\t" // [a_23 a_33]
		"shufpd  $1, %%xmm0, %%xmm0      \n\t" // a_33
		"ucomisd %%xmm7, %%xmm0          \n\t"
		"jbe     .DZERO3                 \n\t"
		"sqrtsd  %%xmm0, %%xmm0          \n\t"
		"movsd   %%xmm0, 120(%%rbx)      \n\t"
		"divsd   %%xmm0, %%xmm6          \n\t" // 1.0/a_11
		"movsd   %%xmm6, 72(%%rax)       \n\t" // fact[9]
		"                                \n\t"
		"jmp    .DEND                    \n\t"
		"                                \n\t"
		"                                \n\t"
		".DZERO0:                        \n\t"
		"xorpd  %%xmm0,  %%xmm0          \n\t"
		"movsd  %%xmm0,  %%xmm9          \n\t"
		"movaps %%xmm9,  (%%rbx)         \n\t"
		"movaps %%xmm13, 16(%%rbx)       \n\t"
		"movsd  %%xmm9,  (%%rax)         \n\t" // fact[0]
		"jmp    .DA11                    \n\t"
		"                                \n\t"
		".DZERO1:                        \n\t"
		"xorpd  %%xmm8,  %%xmm8          \n\t"
//		"movaps %%xmm8,  %%xmm12         \n\t"
		"movsd  %%xmm8,  40(%%rbx)       \n\t"
		"movaps %%xmm12, 48(%%rbx)       \n\t"
		"movsd  %%xmm8,  16(%%rax)       \n\t" // fact[2]
		"jmp    .DA22                    \n\t"
		"                                \n\t"
		".DZERO2:                        \n\t"
		"xorpd  %%xmm0,  %%xmm0          \n\t"
		"movsd  %%xmm0,  %%xmm15         \n\t"
		"movaps %%xmm15, 80(%%rbx)       \n\t"
		"movsd  %%xmm15, 40(%%rax)       \n\t" // fact[5]
		"jmp    .DA33                    \n\t"
		"                                \n\t"
		".DZERO3:                        \n\t"
		"xorpd  %%xmm14, %%xmm14         \n\t"
		"movsd  %%xmm14, 120(%%rbx)      \n\t"
		"movsd  %%xmm14, 72(%%rax)       \n\t" // fact[2]
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"                                \n\t"
		"jmp    .DEND                    \n\t"
		".align 16                       \n\t"
		".DONE:                          \n\t"
		".long  0                        \n\t"
		".long  1072693248               \n\t"
		".long  0                        \n\t"
		".long  0                        \n\t"
		".align 16                       \n\t"
		".DTHR:                          \n\t"
		".long  2665960982               \n\t"
		".long  1020396463               \n\t"
		".long  0                        \n\t"
		".long  0                        \n\t"
		"                                \n\t"
		"                                \n\t"
		".DEND:                          \n\t"
		"                                \n\t"
		: // output operands (none)
		: // input operands
		  "m" (ki_add),		// %0
		  "m" (kl_add),		// %1
		  "m" (ki_sub),		// %2
		  "m" (Ap),			// %3
		  "m" (Bp),			// %4
		  "m" (C),			// %5
		  "m" (D),			// %6
		  "m" (fact),		// %7
		  "m" (alg),		// %8
		  "m" (Am),			// %9
		  "m" (Bm),			// %10
		  "m" (tri),		// %11
		  "m" (km),			// %12
		  "m" (kn)			// %13
		: // register clobber list
		  "rax", "rbx", "rcx", "rdx", "rsi", //"rdx", //"rdi", "r8", "r9", "r10", "r11",
		  "xmm0", "xmm1", "xmm2", "xmm3",
		  "xmm4", "xmm5", "xmm6", "xmm7",
		  "xmm8", "xmm9", "xmm10", "xmm11",
		  "xmm12", "xmm13", "xmm14", "xmm15",
		  "memory"
	);
}



void kernel_dsyrk_dpotrf_nt_4x2_vs_lib4(int km, int kn, int tri, int kadd, int ksub, double *Ap, double *Bp, double *Am, double *Bm, double *C, double *D, double *fact, int alg, int fast_rsqrt)
	{

	const int bs = 4;
//	const int d_ncl = D_NCL;
	const int lda = bs;
	const int ldc = bs;

	int k;

	double
		a_0, a_1, a_2, a_3,
		b_0, b_1,
		c_00=0, 
		c_10=0, c_11=0, 
		c_20=0, c_21=0,  
		c_30=0, c_31=0;

	k = 0;

	if(kadd>0)
		{

		if(tri==1)
			{

			// initial triangle

			if(kadd>=4)
				{
			
				// k = 0
				a_0 = Ap[0+bs*0];
				
				b_0 = Bp[0+bs*0];
					
				c_00 += a_0 * b_0;


				// k = 1
				a_0 = Ap[0+bs*1];
				a_1 = Ap[1+bs*1];
					
				b_0 = Bp[0+bs*1];
				b_1 = Bp[1+bs*1];
					
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;

				c_11 += a_1 * b_1;


				// k = 2
				a_0 = Ap[0+bs*2];
				a_1 = Ap[1+bs*2];
				a_2 = Ap[2+bs*2];
					
				b_0 = Bp[0+bs*2];
				b_1 = Bp[1+bs*2];
					
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;
				c_20 += a_2 * b_0;

				c_11 += a_1 * b_1;
				c_21 += a_2 * b_1;


				// k = 3
				a_0 = Ap[0+bs*3];
				a_1 = Ap[1+bs*3];
				a_2 = Ap[2+bs*3];
				a_3 = Ap[3+bs*3];
					
				b_0 = Bp[0+bs*3];
				b_1 = Bp[1+bs*3];
					
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;
				c_20 += a_2 * b_0;
				c_30 += a_3 * b_0;

				c_11 += a_1 * b_1;
				c_21 += a_2 * b_1;
				c_31 += a_3 * b_1;

				Ap += 16;
				Bp += 16;
				k  += 4;

				}
			else
				{

				// k = 0
				a_0 = Ap[0+bs*0];
				
				b_0 = Bp[0+bs*0];
					
				c_00 += a_0 * b_0;

				k  += 1;

				if(kadd>1)
					{

					// k = 1
					a_0 = Ap[0+bs*1];
					a_1 = Ap[1+bs*1];
						
					b_0 = Bp[0+bs*1];
					b_1 = Bp[1+bs*1];
						
					c_00 += a_0 * b_0;
					c_10 += a_1 * b_0;

					c_11 += a_1 * b_1;

					k  += 1;

					if(kadd>2)
						{

						// k = 2
						a_0 = Ap[0+bs*2];
						a_1 = Ap[1+bs*2];
						a_2 = Ap[2+bs*2];
							
						b_0 = Bp[0+bs*2];
						b_1 = Bp[1+bs*2];
							
						c_00 += a_0 * b_0;
						c_10 += a_1 * b_0;
						c_20 += a_2 * b_0;

						c_11 += a_1 * b_1;
						c_21 += a_2 * b_1;

						k  += 1;

						}

					}

				}

			}

		for(; k<kadd-3; k+=4)
			{
			
			a_0 = Ap[0+bs*0];
			a_1 = Ap[1+bs*0];
			a_2 = Ap[2+bs*0];
			a_3 = Ap[3+bs*0];
			
			b_0 = Bp[0+bs*0];
			b_1 = Bp[1+bs*0];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;
			c_20 += a_2 * b_0;
			c_30 += a_3 * b_0;

			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;


			a_0 = Ap[0+bs*1];
			a_1 = Ap[1+bs*1];
			a_2 = Ap[2+bs*1];
			a_3 = Ap[3+bs*1];
			
			b_0 = Bp[0+bs*1];
			b_1 = Bp[1+bs*1];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;
			c_20 += a_2 * b_0;
			c_30 += a_3 * b_0;

			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;


			a_0 = Ap[0+bs*2];
			a_1 = Ap[1+bs*2];
			a_2 = Ap[2+bs*2];
			a_3 = Ap[3+bs*2];
			
			b_0 = Bp[0+bs*2];
			b_1 = Bp[1+bs*2];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;
			c_20 += a_2 * b_0;
			c_30 += a_3 * b_0;

			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;


			a_0 = Ap[0+bs*3];
			a_1 = Ap[1+bs*3];
			a_2 = Ap[2+bs*3];
			a_3 = Ap[3+bs*3];
			
			b_0 = Bp[0+bs*3];
			b_1 = Bp[1+bs*3];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;
			c_20 += a_2 * b_0;
			c_30 += a_3 * b_0;

			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;
			
			
			Ap += 16;
			Bp += 16;

			}
		for(; k<kadd; k++)
			{
			
			a_0 = Ap[0+bs*0];
			a_1 = Ap[1+bs*0];
			a_2 = Ap[2+bs*0];
			a_3 = Ap[3+bs*0];
			
			b_0 = Bp[0+bs*0];
			b_1 = Bp[1+bs*0];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;
			c_20 += a_2 * b_0;
			c_30 += a_3 * b_0;

			c_11 += a_1 * b_1;
			c_21 += a_2 * b_1;
			c_31 += a_3 * b_1;


			Ap += 4;
			Bp += 4;

			}

		}

	for(k=0; k<ksub-3; k+=4)
		{
		
		a_0 = Am[0+lda*0];
		a_1 = Am[1+lda*0];
		a_2 = Am[2+lda*0];
		a_3 = Am[3+lda*0];
		
		b_0 = Bm[0+lda*0];
		b_1 = Bm[1+lda*0];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;


		a_0 = Am[0+lda*1];
		a_1 = Am[1+lda*1];
		a_2 = Am[2+lda*1];
		a_3 = Am[3+lda*1];
		
		b_0 = Bm[0+lda*1];
		b_1 = Bm[1+lda*1];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;


		a_0 = Am[0+lda*2];
		a_1 = Am[1+lda*2];
		a_2 = Am[2+lda*2];
		a_3 = Am[3+lda*2];
		
		b_0 = Bm[0+lda*2];
		b_1 = Bm[1+lda*2];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;


		a_0 = Am[0+lda*3];
		a_1 = Am[1+lda*3];
		a_2 = Am[2+lda*3];
		a_3 = Am[3+lda*3];
		
		b_0 = Bm[0+lda*3];
		b_1 = Bm[1+lda*3];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;
		c_20 -= a_2 * b_0;
		c_30 -= a_3 * b_0;

		c_11 -= a_1 * b_1;
		c_21 -= a_2 * b_1;
		c_31 -= a_3 * b_1;
		
		
		Am += 16;
		Bm += 16;

		}

	if(alg!=0)
		{
		c_00 += C[0+ldc*0];
		c_10 += C[1+ldc*0];
		c_20 += C[2+ldc*0];
		c_30 += C[3+ldc*0];

		c_11 += C[1+ldc*1];
		c_21 += C[2+ldc*1];
		c_31 += C[3+ldc*1];
		}
	
	// dpotrf
	
	// first column
	if(c_00 > 1e-15)
		{
		c_00 = sqrt(c_00);
		D[0+bs*0] = c_00;
		c_00 = 1.0/c_00;
		fact[0] = c_00;
		c_10 *= c_00;
		c_20 *= c_00;
		c_30 *= c_00;
		}
	else
		{
		c_00 = 0.0;
		D[0+bs*0] = c_00;
		fact[0] = c_00;
		}
	D[1+bs*0] = c_10;
	fact[1] = c_10;
	D[2+bs*0] = c_20;
	fact[3] = c_20;
	if(km>=4)
		{
		D[3+bs*0] = c_30;
		fact[6] = c_30;
		}
	
	if(kn==1)
		return;
	
	// second column
	c_11 -= c_10*c_10;
	c_21 -= c_20*c_10;
	c_31 -= c_30*c_10;
	if(c_11 > 1e-15)
		{
		c_11 = sqrt(c_11);
		D[1+bs*1] = c_11;
		c_11 = 1.0/c_11;
		fact[2] = c_11;
		c_21 *= c_11;
		c_31 *= c_11;
		}
	else
		{
		c_11 = 0.0;
		D[1+bs*1] = c_11;
		fact[2] = c_11;
		}
	D[2+bs*1] = c_21;
	fact[4] = c_21;
	if(km>=4)
		{
		D[3+bs*1] = c_31;
		fact[7] = c_31;
		}

	}



void kernel_dsyrk_dpotrf_nt_2x2_vs_lib4(int km, int kn, int tri, int kadd, int ksub, double *Ap, double *Bp, double *Am, double *Bm, double *C, double *D, double *fact, int alg, int fast_rsqrt)
	{

	const int bs = 4;
//	const int d_ncl = D_NCL;
	const int lda = bs;
	const int ldc = bs;

	int k;

	double
		a_0, a_1,
		b_0, b_1,
		c_00=0, 
		c_10=0, c_11=0;

	k = 0;

	if(kadd>0)
		{
		
		if(tri==1)
			{
		
			// initial triangle

			if(kadd>=2)
				{

				// k = 0
				a_0 = Ap[0+bs*0];
					
				b_0 = Bp[0+bs*0];
					
				c_00 += a_0 * b_0;


				// k = 1
				a_0 = Ap[0+bs*1];
				a_1 = Ap[1+bs*1];
					
				b_0 = Bp[0+bs*1];
				b_1 = Bp[1+bs*1];
					
				c_00 += a_0 * b_0;
				c_10 += a_1 * b_0;

				c_11 += a_1 * b_1;


				Ap += 8;
				Bp += 8;
				k  += 2;

				}
			else
				{
				// k = 0
				a_0 = Ap[0+bs*0];
					
				b_0 = Bp[0+bs*0];
					
				c_00 += a_0 * b_0;

				k  += 1;

				}

			}

		for(; k<kadd-3; k+=4)
			{
			
			a_0 = Ap[0+bs*0];
			a_1 = Ap[1+bs*0];
			
			b_0 = Bp[0+bs*0];
			b_1 = Bp[1+bs*0];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_11 += a_1 * b_1;


			a_0 = Ap[0+bs*1];
			a_1 = Ap[1+bs*1];
			
			b_0 = Bp[0+bs*1];
			b_1 = Bp[1+bs*1];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_11 += a_1 * b_1;


			a_0 = Ap[0+bs*2];
			a_1 = Ap[1+bs*2];
			
			b_0 = Bp[0+bs*2];
			b_1 = Bp[1+bs*2];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_11 += a_1 * b_1;


			a_0 = Ap[0+bs*3];
			a_1 = Ap[1+bs*3];
			
			b_0 = Bp[0+bs*3];
			b_1 = Bp[1+bs*3];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_11 += a_1 * b_1;
			
			
			Ap += 16;
			Bp += 16;

			}
		for(; k<kadd; k++)
			{
			
			a_0 = Ap[0+bs*0];
			a_1 = Ap[1+bs*0];
			
			b_0 = Bp[0+bs*0];
			b_1 = Bp[1+bs*0];
			
			c_00 += a_0 * b_0;
			c_10 += a_1 * b_0;

			c_11 += a_1 * b_1;


			Ap += 4;
			Bp += 4;

			}

		}

	for(k=0; k<ksub-3; k+=4)
		{
		
		a_0 = Am[0+lda*0];
		a_1 = Am[1+lda*0];
		
		b_0 = Bm[0+lda*0];
		b_1 = Bm[1+lda*0];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_11 -= a_1 * b_1;


		a_0 = Am[0+lda*1];
		a_1 = Am[1+lda*1];
		
		b_0 = Bm[0+lda*1];
		b_1 = Bm[1+lda*1];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_11 -= a_1 * b_1;


		a_0 = Am[0+lda*2];
		a_1 = Am[1+lda*2];
		
		b_0 = Bm[0+lda*2];
		b_1 = Bm[1+lda*2];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_11 -= a_1 * b_1;


		a_0 = Am[0+lda*3];
		a_1 = Am[1+lda*3];
		
		b_0 = Bm[0+lda*3];
		b_1 = Bm[1+lda*3];
		
		c_00 -= a_0 * b_0;
		c_10 -= a_1 * b_0;

		c_11 -= a_1 * b_1;
		
		
		Am += 16;
		Bm += 16;

		}

	if(alg!=0)
		{
		c_00 += C[0+ldc*0];
		c_10 += C[1+ldc*0];

		c_11 += C[1+ldc*1];
		}
	
	// dpotrf
	
	// first column
	if(c_00 > 1e-15)
		{
		c_00 = sqrt(c_00);
		D[0+bs*0] = c_00;
		c_00 = 1.0/c_00;
		fact[0] = c_00;
		c_10 *= c_00;
		}
	else
		{
		c_00 = 0.0;
		D[0+bs*0] = c_00;
		fact[0] = c_00;
		}
	if(km>=2)
		{
		D[1+bs*0] = c_10;
		fact[1] = c_10;
		}

	if(kn==1)
		return;
	
	// second column
	c_11 -= c_10*c_10;
	if(c_11 > 1e-15)
		{
		c_11 = sqrt(c_11);
		D[1+bs*1] = c_11;
		c_11 = 1.0/c_11;
		fact[2] = c_11;
		}
	else
		{
		c_11 = 0.0;
		D[1+bs*1] = c_11;
		fact[2] = c_11;
		}

	}



