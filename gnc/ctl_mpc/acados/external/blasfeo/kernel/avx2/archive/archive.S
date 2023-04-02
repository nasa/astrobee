/**************************************************************************************************
*                                                                                                 *
* This file is part of BLASFEO.                                                                   *
*                                                                                                 *
* BLASFEO -- BLAS For Embedded Optimization.                                                      *
* Copyright (C) 2019 by Gianluca Frison.                                                          *
* Developed at IMTEK (University of Freiburg) under the supervision of Moritz Diehl.              *
* All rights reserved.                                                                            *
*                                                                                                 *
* The 2-Clause BSD License                                                                        *
*                                                                                                 *
* Redistribution and use in source and binary forms, with or without                              *
* modification, are permitted provided that the following conditions are met:                     *
*                                                                                                 *
* 1. Redistributions of source code must retain the above copyright notice, this                  *
*    list of conditions and the following disclaimer.                                             *
* 2. Redistributions in binary form must reproduce the above copyright notice,                    *
*    this list of conditions and the following disclaimer in the documentation                    *
*    and/or other materials provided with the distribution.                                       *
*                                                                                                 *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND                 *
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED                   *
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                          *
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR                 *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES                  *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;                    *
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND                     *
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT                      *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS                   *
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                    *
*                                                                                                 *
* Author: Gianluca Frison, gianluca.frison (at) imtek.uni-freiburg.de                             *
*                                                                                                 *
**************************************************************************************************/





//                                       1      2          3          4        5          6        7          8        9                   10      11
// void kernel_dpotrf_nt_l_4x4_vs_lib4ccc(int k, double *A, double *B, int ldb, double *C, int ldc, double *D, int ldd, double *inv_diag_D, int m1, int n1);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dpotrf_nt_l_4x4_vs_lib4ccc
	.type kernel_dpotrf_nt_l_4x4_vs_lib4ccc, @function
kernel_dpotrf_nt_l_4x4_vs_lib4ccc:
#elif defined(OS_MAC)
	.globl _kernel_dpotrf_nt_l_4x4_vs_lib4ccc
_kernel_dpotrf_nt_l_4x4_vs_lib4ccc:
#elif defined(OS_WINDOWS)
	.globl kernel_dpotrf_nt_l_4x4_vs_lib4ccc
	.def kernel_dpotrf_nt_l_4x4_vs_lib4ccc; .scl 2; .type 32; .endef
kernel_dpotrf_nt_l_4x4_vs_lib4ccc:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // B
	movq	ARG4, %r13 // ldb
	sall	$ 3, %r13d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_SUB_NT_4X4_LIB4X
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_sub_nt_4x4_lib4x
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_sub_nt_4x4_lib4x
#endif
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // C
	movq	ARG6, %r11 // ldc
	sall	$ 3, %r11d
	movq	ARG10, %r12 // m1
	movq	ARG11, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_11_4X4_VS_LIB
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_11_4x4_vs_lib
#elif defined(OS_MAC)
	callq _inner_scale_11_4x4_vs_lib
#endif
#endif


	// factorization

	movq	ARG9, %r10  // inv_diag_D 
	movq	ARG11, %r11 // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_4X4_VS_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_dpotrf_4x4_vs_lib4
#elif defined(OS_MAC)
	callq _inner_edge_dpotrf_4x4_vs_lib4
#endif
#endif


	// store

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG10, %r12 // m1
	movq	ARG11, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_VS_LIB
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_l_4x4_vs_lib
#elif defined(OS_MAC)
	callq _inner_store_l_4x4_vs_lib
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dpotrf_nt_l_4x4_vs_lib4ccc, .-kernel_dpotrf_nt_l_4x4_vs_lib4ccc
#endif





//                                    1      2          3          4        5          6        7          8        9
// void kernel_dpotrf_nt_l_4x4_lib4ccc(int k, double *A, double *B, int ldb, double *C, int ldc, double *D, int ldd, double *inv_diag_D);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dpotrf_nt_l_4x4_lib4ccc
	.type kernel_dpotrf_nt_l_4x4_lib4ccc, @function
kernel_dpotrf_nt_l_4x4_lib4ccc:
#elif defined(OS_MAC)
	.globl _kernel_dpotrf_nt_l_4x4_lib4ccc
_kernel_dpotrf_nt_l_4x4_lib4ccc:
#elif defined(OS_WINDOWS)
	.globl kernel_dpotrf_nt_l_4x4_lib4ccc
	.def kernel_dpotrf_nt_l_4x4_lib4ccc; .scl 2; .type 32; .endef
kernel_dpotrf_nt_l_4x4_lib4ccc:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // B
	movq	ARG4, %r13 // ldb
	sall	$ 3, %r13d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_SUB_NT_4X4_LIB4X
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_sub_nt_4x4_lib4x
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_sub_nt_4x4_lib4x
#endif
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // C
	movq	ARG6, %r11 // ldc
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_SCALE_11_4X4_LIB
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_11_4x4_lib
#elif defined(OS_MAC)
	callq _inner_scale_11_4x4_lib
#endif
#endif


	// factorization

	movq	ARG9, %r10  // inv_diag_D 
	movl	$ 4, %r11d

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_4X4_VS_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_dpotrf_4x4_vs_lib4
#elif defined(OS_MAC)
	callq _inner_edge_dpotrf_4x4_vs_lib4
#endif
#endif


	// store

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_LIB
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_l_4x4_lib
#elif defined(OS_MAC)
	callq _inner_store_l_4x4_lib
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dpotrf_nt_l_4x4_lib4ccc, .-kernel_dpotrf_nt_l_4x4_lib4ccc
#endif





//                                    1      2          3        4          5        6          7        8          9        10
// void kernel_dpotrf_nt_l_8x4_lib4ccc(int k, double *A, int sda, double *B, int ldb, double *C, int ldc, double *D, int ldd, double *inv_diag_D);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dpotrf_nt_l_8x4_lib4ccc
	.type kernel_dpotrf_nt_l_8x4_lib4ccc, @function
kernel_dpotrf_nt_l_8x4_lib4ccc:
#elif defined(OS_MAC)
	.globl _kernel_dpotrf_nt_l_8x4_lib4ccc
_kernel_dpotrf_nt_l_8x4_lib4ccc:
#elif defined(OS_WINDOWS)
	.globl kernel_dpotrf_nt_l_8x4_lib4ccc
	.def kernel_dpotrf_nt_l_8x4_lib4ccc; .scl 2; .type 32; .endef
kernel_dpotrf_nt_l_8x4_lib4ccc:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3
	vmovapd	%ymm0, %ymm4
	vmovapd	%ymm0, %ymm5
	vmovapd	%ymm0, %ymm6
	vmovapd	%ymm0, %ymm7


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // ldb
	sall	$ 5, %r12d
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_SUB_NT_8X4_LIB4X
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_sub_nt_8x4_lib4x
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_sub_nt_8x4_lib4x
#endif
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // C
	movq	ARG7, %r11 // ldc
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_SCALE_11_8X4_LIB
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_11_8x4_lib
#elif defined(OS_MAC)
	callq _inner_scale_11_8x4_lib
#endif
#endif


	// factorization

	movq	ARG10, %r10  // inv_diag_D 
	movl	$ 4, %r11d

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_8X4_VS_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_dpotrf_8x4_vs_lib4
#elif defined(OS_MAC)
	callq _inner_edge_dpotrf_8x4_vs_lib4
#endif
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X4_LIB
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_l_8x4_lib
#elif defined(OS_MAC)
	callq _inner_store_l_8x4_lib
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dpotrf_nt_l_8x4_lib4ccc, .-kernel_dpotrf_nt_l_8x4_lib4ccc
#endif





//                                     1      2          3        4          5        6          7        8          9        10
// void kernel_dpotrf_nt_l_12x4_lib4ccc(int k, double *A, int sda, double *B, int ldb, double *C, int ldc, double *D, int ldd, double *inv_diag_D);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dpotrf_nt_l_12x4_lib4ccc
	.type kernel_dpotrf_nt_l_12x4_lib4ccc, @function
kernel_dpotrf_nt_l_12x4_lib4ccc:
#elif defined(OS_MAC)
	.globl _kernel_dpotrf_nt_l_12x4_lib4ccc
_kernel_dpotrf_nt_l_12x4_lib4ccc:
#elif defined(OS_WINDOWS)
	.globl kernel_dpotrf_nt_l_12x4_lib4ccc
	.def kernel_dpotrf_nt_l_12x4_lib4ccc; .scl 2; .type 32; .endef
kernel_dpotrf_nt_l_12x4_lib4ccc:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3
	vmovapd	%ymm0, %ymm4
	vmovapd	%ymm0, %ymm5
	vmovapd	%ymm0, %ymm6
	vmovapd	%ymm0, %ymm7
	vmovapd	%ymm0, %ymm8
	vmovapd	%ymm0, %ymm9
	vmovapd	%ymm0, %ymm10
	vmovapd	%ymm0, %ymm11


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // ldb
	sall	$ 5, %r12d
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_SUB_NT_12X4_LIB4X
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_sub_nt_12x4_lib4x
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_sub_nt_12x4_lib4x
#endif
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // C
	movq	ARG7, %r11 // ldc
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_SCALE_11_12X4_LIB
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_11_12x4_lib
#elif defined(OS_MAC)
	callq _inner_scale_11_12x4_lib
#endif
#endif


	// factorization

	movq	ARG10, %r10  // inv_diag_D 
	movl	$ 4, %r11d

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_12X4_VS_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_dpotrf_12x4_vs_lib4
#elif defined(OS_MAC)
	callq _inner_edge_dpotrf_12x4_vs_lib4
#endif
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_L_12X4_LIB
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_l_12x4_lib
#elif defined(OS_MAC)
	callq _inner_store_l_12x4_lib
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dpotrf_nt_l_12x4_lib4ccc, .-kernel_dpotrf_nt_l_12x4_lib4ccc
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// r13   <- ldb
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_SUB_NT_4X4_LIB4X
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_dgemm_sub_nt_4x4_lib4x, @function
inner_kernel_dgemm_sub_nt_4x4_lib4x:
#elif defined(OS_MAC)
_inner_kernel_dgemm_sub_nt_4x4_lib4x:
#elif defined(OS_WINDOWS)
	.def inner_kernel_dgemm_sub_nt_4x4_lib4x; .scl 2; .type 32; .endef
inner_kernel_dgemm_sub_nt_4x4_lib4x:
#endif
#endif

	cmpl	$ 0, %r10d
	jle		5f // return

	// preload

	vxorpd			%ymm4, %ymm4, %ymm4
	vmovapd			%ymm4, %ymm5
	vmovapd			%ymm4, %ymm6
	vmovapd			%ymm4, %ymm7

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	0(%r12, %r13, 2) // software prefetch
//	prefetcht0	64(%r12, %r13, 2) // software prefetch

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm3
	addq	%r13, %r12

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm4
	vbroadcastsd	8(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm5
	vbroadcastsd	16(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm6
	vbroadcastsd	24(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm7
	addq	%r13, %r12

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm3
	addq	%r13, %r12

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm4
	vbroadcastsd	8(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm5
	vbroadcastsd	16(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm6
	vbroadcastsd	24(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm7
	addq	%r13, %r12

	subl	$ 4, %r10d
	addq	$ 128, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm3
	addq	%r13, %r12

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm4
	vbroadcastsd	8(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm5
	vbroadcastsd	16(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm6
	vbroadcastsd	24(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm7
	addq	%r13, %r12

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm3
	addq	%r13, %r12

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm4
	vbroadcastsd	8(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm5
	vbroadcastsd	16(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm6
	vbroadcastsd	24(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm7
	addq	%r13, %r12

	subl	$ 4, %r10d
	addq	$ 128, %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm3
	addq	%r13, %r12

	subl	$ 1, %r10d
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

	vaddpd			%ymm4, %ymm0, %ymm0
	vaddpd			%ymm5, %ymm1, %ymm1
	vaddpd			%ymm6, %ymm2, %ymm2
	vaddpd			%ymm7, %ymm3, %ymm3

5: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_dgemm_sub_nt_4x4_lib4x, .-inner_kernel_dgemm_sub_nt_4x4_lib4x
#endif
#endif





// common inner routine with file scope
//
// scale for alpha=1 and beta=1
//
// input arguments:
// r10   <- C
// r11   <- ldc
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_11_4X4_LIB
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_11_4x4_lib, @function
inner_scale_11_4x4_lib:
#elif defined(OS_MAC)
_inner_scale_11_4x4_lib:
#elif defined(OS_WINDOWS)
	.def inner_scale_11_4x4_lib; .scl 2; .type 32; .endef
inner_scale_11_4x4_lib:
#endif
#endif

	vmovupd		0(%r10), %ymm15
	vaddpd		%ymm0, %ymm15, %ymm0
	addq		%r11, %r10
	vmovupd		0(%r10), %ymm15
	vaddpd		%ymm1, %ymm15, %ymm1
	addq		%r11, %r10
	vmovupd		0(%r10), %ymm15
	vaddpd		%ymm2, %ymm15, %ymm2
	addq		%r11, %r10
	vmovupd		0(%r10), %ymm15
	vaddpd		%ymm3, %ymm15, %ymm3
//	addq		%r11, %r10

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_scale_11_4x4_lib, .-inner_scale_11_4x4_lib
#endif
#endif





// common inner routine with file scope
//
// scale for alpha=1 and beta=1
//
// input arguments:
// r10   <- C
// r11   <- ldc
// r12d   <- km
// r13d   <- kn
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_11_4X4_VS_LIB
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_11_4x4_vs_lib, @function
inner_scale_11_4x4_vs_lib:
#elif defined(OS_MAC)
_inner_scale_11_4x4_vs_lib:
#elif defined(OS_WINDOWS)
	.def inner_scale_11_4x4_vs_lib; .scl 2; .type 32; .endef
inner_scale_11_4x4_vs_lib:
#endif
#endif

	vcvtsi2sd	%r12d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC00(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC00(%rip), %ymm13
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm13, %ymm13


	vmaskmovpd	0(%r10), %ymm13, %ymm15
	vaddpd		%ymm0, %ymm15, %ymm0
	addq		%r11, %r10
	cmpl		$ 2, %r13d
	jl			0f // end
	vmaskmovpd	0(%r10), %ymm13, %ymm15
	vaddpd		%ymm1, %ymm15, %ymm1
	addq		%r11, %r10
	cmpl		$ 3, %r13d
	jl			0f // end
	vmaskmovpd	0(%r10), %ymm13, %ymm15
	vaddpd		%ymm2, %ymm15, %ymm2
	addq		%r11, %r10
	cmpl		$ 3, %r13d
	je			0f // end
	vmaskmovpd	0(%r10), %ymm13, %ymm15
	vaddpd		%ymm3, %ymm15, %ymm3
//	addq		%r11, %r10

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_scale_11_4x4_vs_lib, .-inner_scale_11_4x4_vs_lib
#endif
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// r14   <- ldb
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_SUB_NT_8X4_LIB4X
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_dgemm_sub_nt_8x4_lib4x, @function
inner_kernel_dgemm_sub_nt_8x4_lib4x:
#elif defined(OS_MAC)
_inner_kernel_dgemm_sub_nt_8x4_lib4x:
#elif defined(OS_WINDOWS)
	.def inner_kernel_dgemm_sub_nt_8x4_lib4x; .scl 2; .type 32; .endef
inner_kernel_dgemm_sub_nt_8x4_lib4x:
#endif
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	0(%r12, %r13, 2) // software prefetch
//	prefetcht0	64(%r12, %r13, 2) // software prefetch

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12, 1), %ymm14 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm0
	vfnmadd231pd	%ymm14, %ymm12, %ymm4
	vbroadcastsd	8(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm1
	vfnmadd231pd	%ymm14, %ymm12, %ymm5
	vbroadcastsd	16(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm2
	vfnmadd231pd	%ymm14, %ymm12, %ymm6
	vbroadcastsd	24(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm3
	vfnmadd231pd	%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12, 1), %ymm14 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm0
	vfnmadd231pd	%ymm14, %ymm12, %ymm4
	vbroadcastsd	8(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm1
	vfnmadd231pd	%ymm14, %ymm12, %ymm5
	vbroadcastsd	16(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm2
	vfnmadd231pd	%ymm14, %ymm12, %ymm6
	vbroadcastsd	24(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm3
	vfnmadd231pd	%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vmovupd			64(%r11, %r12, 1), %ymm14 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm0
	vfnmadd231pd	%ymm14, %ymm12, %ymm4
	vbroadcastsd	8(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm1
	vfnmadd231pd	%ymm14, %ymm12, %ymm5
	vbroadcastsd	16(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm2
	vfnmadd231pd	%ymm14, %ymm12, %ymm6
	vbroadcastsd	24(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm3
	vfnmadd231pd	%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vmovupd			96(%r11, %r12, 1), %ymm14 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm0
	vfnmadd231pd	%ymm14, %ymm12, %ymm4
	vbroadcastsd	8(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm1
	vfnmadd231pd	%ymm14, %ymm12, %ymm5
	vbroadcastsd	16(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm2
	vfnmadd231pd	%ymm14, %ymm12, %ymm6
	vbroadcastsd	24(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm3
	vfnmadd231pd	%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 128, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12, 1), %ymm14 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm0
	vfnmadd231pd	%ymm14, %ymm12, %ymm4
	vbroadcastsd	8(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm1
	vfnmadd231pd	%ymm14, %ymm12, %ymm5
	vbroadcastsd	16(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm2
	vfnmadd231pd	%ymm14, %ymm12, %ymm6
	vbroadcastsd	24(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm3
	vfnmadd231pd	%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12, 1), %ymm14 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm0
	vfnmadd231pd	%ymm14, %ymm12, %ymm4
	vbroadcastsd	8(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm1
	vfnmadd231pd	%ymm14, %ymm12, %ymm5
	vbroadcastsd	16(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm2
	vfnmadd231pd	%ymm14, %ymm12, %ymm6
	vbroadcastsd	24(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm3
	vfnmadd231pd	%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vmovupd			64(%r11, %r12, 1), %ymm14 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm0
	vfnmadd231pd	%ymm14, %ymm12, %ymm4
	vbroadcastsd	8(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm1
	vfnmadd231pd	%ymm14, %ymm12, %ymm5
	vbroadcastsd	16(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm2
	vfnmadd231pd	%ymm14, %ymm12, %ymm6
	vbroadcastsd	24(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm3
	vfnmadd231pd	%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vmovupd			96(%r11, %r12, 1), %ymm14 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm0
	vfnmadd231pd	%ymm14, %ymm12, %ymm4
	vbroadcastsd	8(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm1
	vfnmadd231pd	%ymm14, %ymm12, %ymm5
	vbroadcastsd	16(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm2
	vfnmadd231pd	%ymm14, %ymm12, %ymm6
	vbroadcastsd	24(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm3
	vfnmadd231pd	%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 128, %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12, 1), %ymm14 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm0
	vfnmadd231pd	%ymm14, %ymm12, %ymm4
	vbroadcastsd	8(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm1
	vfnmadd231pd	%ymm14, %ymm12, %ymm5
	vbroadcastsd	16(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm2
	vfnmadd231pd	%ymm14, %ymm12, %ymm6
	vbroadcastsd	24(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm3
	vfnmadd231pd	%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	subl	$ 1, %r10d
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_dgemm_sub_nt_8x4_lib4x, .-inner_kernel_dgemm_sub_nt_8x4_lib4x
#endif
#endif





// common inner routine with file scope
//
// scale for alpha=1 and beta=1
//
// input arguments:
// r10   <- C
// r11   <- ldc
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_11_8X4_LIB
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_11_8x4_lib, @function
inner_scale_11_8x4_lib:
#elif defined(OS_MAC)
_inner_scale_11_8x4_lib:
#elif defined(OS_WINDOWS)
	.def inner_scale_11_8x4_lib; .scl 2; .type 32; .endef
inner_scale_11_8x4_lib:
#endif
#endif

	vmovupd		0(%r10), %ymm15
	vaddpd		%ymm0, %ymm15, %ymm0
	vmovupd		32(%r10), %ymm15
	vaddpd		%ymm4, %ymm15, %ymm4
	addq		%r11, %r10
	vmovupd		0(%r10), %ymm15
	vaddpd		%ymm1, %ymm15, %ymm1
	vmovupd		32(%r10), %ymm15
	vaddpd		%ymm5, %ymm15, %ymm5
	addq		%r11, %r10
	vmovupd		0(%r10), %ymm15
	vaddpd		%ymm2, %ymm15, %ymm2
	vmovupd		32(%r10), %ymm15
	vaddpd		%ymm6, %ymm15, %ymm6
	addq		%r11, %r10
	vmovupd		0(%r10), %ymm15
	vaddpd		%ymm3, %ymm15, %ymm3
	vmovupd		32(%r10), %ymm15
	vaddpd		%ymm7, %ymm15, %ymm7
//	addq		%r11, %r10

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_scale_11_8x4_lib, .-inner_scale_11_8x4_lib
#endif
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// r14   <- ldb
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- [840 d90 da0 db0]
// ymm9  <- [841 d91 da1 db1]
// ymm10 <- [842 d92 da2 db2]
// ymm11 <- [843 d93 da3 db3]
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_SUB_NT_12X4_LIB4X
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_dgemm_sub_nt_12x4_lib4x, @function
inner_kernel_dgemm_sub_nt_12x4_lib4x:
#elif defined(OS_MAC)
_inner_kernel_dgemm_sub_nt_12x4_lib4x:
#elif defined(OS_WINDOWS)
	.def inner_kernel_dgemm_sub_nt_12x4_lib4x; .scl 2; .type 32; .endef
inner_kernel_dgemm_sub_nt_12x4_lib4x:
#endif
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	0(%r12, %r13, 2) // software prefetch
//	prefetcht0	64(%r12, %r13, 2) // software prefetch

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12, 1), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm0
	vfnmadd231pd	%ymm14, %ymm12, %ymm4
	vfnmadd231pd	%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm1
	vfnmadd231pd	%ymm14, %ymm12, %ymm5
	vfnmadd231pd	%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm2
	vfnmadd231pd	%ymm14, %ymm12, %ymm6
	vfnmadd231pd	%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm3
	vfnmadd231pd	%ymm14, %ymm12, %ymm7
	vfnmadd231pd	%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12, 1), %ymm14 // A
	vmovupd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm0
	vfnmadd231pd	%ymm14, %ymm12, %ymm4
	vfnmadd231pd	%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm1
	vfnmadd231pd	%ymm14, %ymm12, %ymm5
	vfnmadd231pd	%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm2
	vfnmadd231pd	%ymm14, %ymm12, %ymm6
	vfnmadd231pd	%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm3
	vfnmadd231pd	%ymm14, %ymm12, %ymm7
	vfnmadd231pd	%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vmovupd			64(%r11, %r12, 1), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm0
	vfnmadd231pd	%ymm14, %ymm12, %ymm4
	vfnmadd231pd	%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm1
	vfnmadd231pd	%ymm14, %ymm12, %ymm5
	vfnmadd231pd	%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm2
	vfnmadd231pd	%ymm14, %ymm12, %ymm6
	vfnmadd231pd	%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm3
	vfnmadd231pd	%ymm14, %ymm12, %ymm7
	vfnmadd231pd	%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vmovupd			96(%r11, %r12, 1), %ymm14 // A
	vmovupd			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm0
	vfnmadd231pd	%ymm14, %ymm12, %ymm4
	vfnmadd231pd	%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm1
	vfnmadd231pd	%ymm14, %ymm12, %ymm5
	vfnmadd231pd	%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm2
	vfnmadd231pd	%ymm14, %ymm12, %ymm6
	vfnmadd231pd	%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm3
	vfnmadd231pd	%ymm14, %ymm12, %ymm7
	vfnmadd231pd	%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 128, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12, 1), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm0
	vfnmadd231pd	%ymm14, %ymm12, %ymm4
	vfnmadd231pd	%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm1
	vfnmadd231pd	%ymm14, %ymm12, %ymm5
	vfnmadd231pd	%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm2
	vfnmadd231pd	%ymm14, %ymm12, %ymm6
	vfnmadd231pd	%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm3
	vfnmadd231pd	%ymm14, %ymm12, %ymm7
	vfnmadd231pd	%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12, 1), %ymm14 // A
	vmovupd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm0
	vfnmadd231pd	%ymm14, %ymm12, %ymm4
	vfnmadd231pd	%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm1
	vfnmadd231pd	%ymm14, %ymm12, %ymm5
	vfnmadd231pd	%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm2
	vfnmadd231pd	%ymm14, %ymm12, %ymm6
	vfnmadd231pd	%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm3
	vfnmadd231pd	%ymm14, %ymm12, %ymm7
	vfnmadd231pd	%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vmovupd			64(%r11, %r12, 1), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm0
	vfnmadd231pd	%ymm14, %ymm12, %ymm4
	vfnmadd231pd	%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm1
	vfnmadd231pd	%ymm14, %ymm12, %ymm5
	vfnmadd231pd	%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm2
	vfnmadd231pd	%ymm14, %ymm12, %ymm6
	vfnmadd231pd	%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm3
	vfnmadd231pd	%ymm14, %ymm12, %ymm7
	vfnmadd231pd	%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vmovupd			96(%r11, %r12, 1), %ymm14 // A
	vmovupd			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm0
	vfnmadd231pd	%ymm14, %ymm12, %ymm4
	vfnmadd231pd	%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm1
	vfnmadd231pd	%ymm14, %ymm12, %ymm5
	vfnmadd231pd	%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm2
	vfnmadd231pd	%ymm14, %ymm12, %ymm6
	vfnmadd231pd	%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm3
	vfnmadd231pd	%ymm14, %ymm12, %ymm7
	vfnmadd231pd	%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 128, %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12, 1), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm0
	vfnmadd231pd	%ymm14, %ymm12, %ymm4
	vfnmadd231pd	%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm1
	vfnmadd231pd	%ymm14, %ymm12, %ymm5
	vfnmadd231pd	%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm2
	vfnmadd231pd	%ymm14, %ymm12, %ymm6
	vfnmadd231pd	%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfnmadd231pd	%ymm13, %ymm12, %ymm3
	vfnmadd231pd	%ymm14, %ymm12, %ymm7
	vfnmadd231pd	%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	subl	$ 1, %r10d
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_dgemm_sub_nt_12x4_lib4x, .-inner_kernel_dgemm_sub_nt_12x4_lib4x
#endif
#endif





// common inner routine with file scope
//
// scale for alpha=1 and beta=1
//
// input arguments:
// r10   <- C
// r11   <- ldc
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- [840 d90 da0 db0]
// ymm9  <- [841 d91 da1 db1]
// ymm10 <- [842 d92 da2 db2]
// ymm11 <- [843 d93 da3 db3]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_11_12X4_LIB
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_11_12x4_lib, @function
inner_scale_11_12x4_lib:
#elif defined(OS_MAC)
_inner_scale_11_12x4_lib:
#elif defined(OS_WINDOWS)
	.def inner_scale_11_12x4_lib; .scl 2; .type 32; .endef
inner_scale_11_12x4_lib:
#endif
#endif

	vmovupd		0(%r10), %ymm15
	vaddpd		%ymm0, %ymm15, %ymm0
	vmovupd		32(%r10), %ymm15
	vaddpd		%ymm4, %ymm15, %ymm4
	vmovupd		64(%r10), %ymm15
	vaddpd		%ymm8, %ymm15, %ymm8
	addq		%r11, %r10
	vmovupd		0(%r10), %ymm15
	vaddpd		%ymm1, %ymm15, %ymm1
	vmovupd		32(%r10), %ymm15
	vaddpd		%ymm5, %ymm15, %ymm5
	vmovupd		64(%r10), %ymm15
	vaddpd		%ymm9, %ymm15, %ymm9
	addq		%r11, %r10
	vmovupd		0(%r10), %ymm15
	vaddpd		%ymm2, %ymm15, %ymm2
	vmovupd		32(%r10), %ymm15
	vaddpd		%ymm6, %ymm15, %ymm6
	vmovupd		64(%r10), %ymm15
	vaddpd		%ymm10, %ymm15, %ymm10
	addq		%r11, %r10
	vmovupd		0(%r10), %ymm15
	vaddpd		%ymm3, %ymm15, %ymm3
	vmovupd		32(%r10), %ymm15
	vaddpd		%ymm7, %ymm15, %ymm7
	vmovupd		64(%r10), %ymm15
	vaddpd		%ymm11, %ymm15, %ymm11
//	addq		%r11, %r10

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_scale_11_12x4_lib, .-inner_scale_11_12x4_lib
#endif
#endif






