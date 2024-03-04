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



// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- lda*sizeof(double)
// r13   <- z_n
// ymm0  <- x_n_0
// ymm1  <- x_n_1
// ymm2  <- x_n_2
// ymm3  <- x_n_3

//
// output arguments:
// r10d  <- 0
// r11   <- A+4*k*sizeof(double)
// r12   <- lda*sizeof(double)
// r13   <- z_n+k*sizeof(double)
// ymm0  <- x_n_0
// ymm1  <- x_n_1
// ymm2  <- x_n_2
// ymm3  <- x_n_3

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMV_N_4_LIBC
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemv_n_4_libc)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	leaq	0(%r12, %r12, 2), %r14

	cmpl	$ 4, %r10d
	jl		0f // clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	vmovupd	0(%r13), %ymm8

	vmovupd		0(%r11), %ymm14
	vfmadd231pd	%ymm14, %ymm0, %ymm8

	subl	$ 4, %r10d

	vmovupd	0(%r11, %r12), %ymm14
	vfmadd231pd	%ymm14, %ymm1, %ymm8
	
	vmovupd	0(%r11, %r12, 2), %ymm14
	vfmadd231pd	%ymm14, %ymm2, %ymm8

	vmovupd	0(%r11, %r14), %ymm14
	vfmadd231pd	%ymm14, %ymm3, %ymm8

	vmovupd	%ymm8, 0(%r13) 

	addq	$ 32, %r11
	addq	$ 32, %r13
	
	cmpl	$ 3, %r10d

	jg		1b // main loop 


	// consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

0: // clean-up
	
	vcvtsi2sd	%r10d, %xmm14, %xmm14
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %ymm13
#endif
	vmovddup	%xmm14, %xmm14
	vinsertf128	$ 1, %xmm14, %ymm14, %ymm14
	vsubpd		%ymm14, %ymm13, %ymm11

	vmaskmovpd	0(%r13), %ymm11, %ymm8

	vmaskmovpd	0(%r11), %ymm11, %ymm14
	vfmadd231pd	%ymm14, %ymm0, %ymm8
	
	vmaskmovpd	0(%r11, %r12), %ymm11, %ymm14
	vfmadd231pd	%ymm14, %ymm1, %ymm8
	
	vmaskmovpd	0(%r11, %r12, 2), %ymm11, %ymm14
	vfmadd231pd	%ymm14, %ymm2, %ymm8

	vmaskmovpd	0(%r11, %r14), %ymm11, %ymm14
	vfmadd231pd	%ymm14, %ymm3, %ymm8
		
	vmaskmovpd	%ymm8, %ymm11, 0(%r13)

	sall	$ 3, %r10d // *sizeof(double)
	addq	%r10, %r11
	addq	%r10, %r13
	xorl	%r10d, %r10d
	
	
2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemv_n_4_libc)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- lda*sizeof(double)
// r13   <- x
// ymm0  <- [z0a z0b z0c z0d]
// ymm1  <- [z1a z1b z1c z1d]
// ymm2  <- [z2a z2b z2c z2d]
// ymm3  <- [z3a z3b z3c z3d]

//
// output arguments:
// r10d  <- 0
// r11   <- A+k*sizeof(double)
// r12   <- lda*sizeof(double)
// r13   <- x+k*sizeof(double)
// ymm0  <- [z0a z0b z0c z0d]
// ymm1  <- [z1a z1b z1c z1d]
// ymm2  <- [z2a z2b z2c z2d]
// ymm3  <- [z3a z3b z3c z3d]

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMV_T_4_LIBC
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemv_t_4_libc)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	leaq	0(%r12, %r12, 2), %r14

	cmpl	$ 4, %r10d
	jl		0f // clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	vmovupd	0(%r13), %ymm12

	vmovupd	0(%r11), %ymm8
	vfmadd231pd	%ymm8, %ymm12, %ymm0
	
	subl	$ 4, %r10d

	vmovupd	0(%r11, %r12), %ymm8
	vfmadd231pd	%ymm8, %ymm12, %ymm1
	
	vmovupd	0(%r11, %r12, 2), %ymm8
	vfmadd231pd	%ymm8, %ymm12, %ymm2

	vmovupd	0(%r11, %r14), %ymm8
	vfmadd231pd	%ymm8, %ymm12, %ymm3
	
	addq	$ 32, %r11
	addq	$ 32, %r13
	
	cmpl	$ 3, %r10d

	jg		1b // main loop 


	// consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

0: // clean-up
	
	vcvtsi2sd	%r10d, %xmm14, %xmm14
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %ymm13
#endif
	vmovddup	%xmm14, %xmm14
	vinsertf128	$ 1, %xmm14, %ymm14, %ymm14
	vsubpd		%ymm14, %ymm13, %ymm14

	vmaskmovpd	0(%r13), %ymm14, %ymm12

	vmaskmovpd	0(%r11), %ymm14, %ymm8
	vfmadd231pd	%ymm8, %ymm12, %ymm0
	
	vmaskmovpd	0(%r11, %r12), %ymm14, %ymm8
	vfmadd231pd	%ymm8, %ymm12, %ymm1
	
	vmaskmovpd	0(%r11, %r12, 2), %ymm14, %ymm8
	vfmadd231pd	%ymm8, %ymm12, %ymm2

	vmaskmovpd	0(%r11, %r14), %ymm14, %ymm8
	vfmadd231pd	%ymm8, %ymm12, %ymm3
		
	sall	$ 3, %r10d
//	movslq	%r10d, %r10
	addq	%r10, %r11
	addq	%r10, %r13
	xorl	%r10d, %r10d
	
	
2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemv_t_4_libc)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- lda*sizeof(double)
// r13   <- x_t
// r14   <- z_n
// ymm0  <- [z_t_0a z_t_0b z_t_0c z_t_0d]
// ymm1  <- [z_t_1a z_t_1b z_t_1c z_t_1d]
// ymm2  <- [z_t_2a z_t_2b z_t_2c z_t_2d]
// ymm3  <- [z_t_3a z_t_3b z_t_3c z_t_3d]
// ymm6  <- x_n_0
// ymm7  <- x_n_1
// ymm8  <- x_n_2
// ymm9  <- x_n_3

//
// output arguments:
// r10d  <- 0
// r11   <- A+4*k*sizeof(double)
// r12   <- lda*sizeof(double)
// r13   <- x_t+k*sizeof(double)
// r14   <- z_n+k*sizeof(double)
// ymm0  <- [z_t_0a z_t_0b z_t_0c z_t_0d]
// ymm1  <- [z_t_1a z_t_1b z_t_1c z_t_1d]
// ymm2  <- [z_t_2a z_t_2b z_t_2c z_t_2d]
// ymm3  <- [z_t_3a z_t_3b z_t_3c z_t_3d]
// ymm6  <- x_n_0
// ymm7  <- x_n_1
// ymm8  <- x_n_2
// ymm9  <- x_n_3

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMV_NT_4_LIBC
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemv_nt_4_libc)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	leaq	0(%r12, %r12, 2), %r15

	cmpl	$ 4, %r10d
	jl		0f // clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	vmovupd	0(%r13), %ymm12
	vmovupd	0(%r14), %ymm13

	vmovupd		0(%r11), %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm0
	vfmadd231pd	%ymm14, %ymm6, %ymm13

	subl	$ 4, %r10d

	vmovupd	0(%r11, %r12), %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm1
	vfmadd231pd	%ymm14, %ymm7, %ymm13
	
	vmovupd	0(%r11, %r12, 2), %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm2
	vfmadd231pd	%ymm14, %ymm8, %ymm13

	vmovupd	0(%r11, %r15), %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm3
	vfmadd231pd	%ymm14, %ymm9, %ymm13

	vmovupd	%ymm13, 0(%r14) 

	addq	$ 32, %r11
	addq	$ 32, %r13
	addq	$ 32, %r14
	
	cmpl	$ 3, %r10d

	jg		1b // main loop 


	// consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

0: // clean-up
	
	vcvtsi2sd	%r10d, %xmm14, %xmm14
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %ymm13
#endif
	vmovddup	%xmm14, %xmm14
	vinsertf128	$ 1, %xmm14, %ymm14, %ymm14
	vsubpd		%ymm14, %ymm13, %ymm11

	vmaskmovpd	0(%r13), %ymm11, %ymm12
	vmaskmovpd	0(%r14), %ymm11, %ymm13

//	vmovupd	%ymm14, -32(%rsp) // spill mask to stack

//	vmovupd	-32(%rsp), %ymm14 // load mask form stack
	vmaskmovpd	0(%r11), %ymm11, %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm0
	vfmadd231pd	%ymm14, %ymm6, %ymm13
	
//	vmovupd	-32(%rsp), %ymm14 // load mask form stack
	vmaskmovpd	0(%r11, %r12), %ymm11, %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm1
	vfmadd231pd	%ymm14, %ymm7, %ymm13
	
//	vmovupd	-32(%rsp), %ymm14 // load mask form stack
	vmaskmovpd	0(%r11, %r12, 2), %ymm11, %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm2
	vfmadd231pd	%ymm14, %ymm8, %ymm13

//	vmovupd	-32(%rsp), %ymm14 // load mask form stack
	vmaskmovpd	0(%r11, %r15), %ymm11, %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm3
	vfmadd231pd	%ymm14, %ymm9, %ymm13
		
//	vmovupd	-32(%rsp), %ymm14 // load mask form stack
	vmaskmovpd	%ymm13, %ymm11, 0(%r14)

	sall	$ 3, %r10d // *sizeof(double)
	addq	%r10, %r11
	addq	%r10, %r13
	addq	%r10, %r14
	xorl	%r10d, %r10d
	
	
2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemv_nt_4_libc)
#endif





// common inner routine with file scope
//
// input arguments:
// r10   <- kmax
// r11   <- A
// r12   <- bs*sda*sizeof(double) = 32*sda
// r13   <- x_t
// r14   <- z_n
// ymm0  <- [z_t_0a z_t_0b z_t_0c z_t_0d]
// ymm1  <- [z_t_1a z_t_1b z_t_1c z_t_1d]
// ymm2  <- [z_t_2a z_t_2b z_t_2c z_t_2d]
// ymm3  <- [z_t_3a z_t_3b z_t_3c z_t_3d]
// ymm6  <- x_n_0
// ymm7  <- x_n_1
// ymm8  <- x_n_2
// ymm9  <- x_n_3

//
// output arguments:
// r10   <- kmax-4
// r11   <- A+4*k*sizeof(double)
// r12   <- bs*sda*sizeof(double) = 32*sda
// r13   <- x_t+k*sizeof(double)
// r14   <- z_n+k*sizeof(double)
// ymm0  <- [z_t_0a z_t_0b z_t_0c z_t_0d]
// ymm1  <- [z_t_1a z_t_1b z_t_1c z_t_1d]
// ymm2  <- [z_t_2a z_t_2b z_t_2c z_t_2d]
// ymm3  <- [z_t_3a z_t_3b z_t_3c z_t_3d]
// ymm6  <- x_n_0
// ymm7  <- x_n_1
// ymm8  <- x_n_2
// ymm9  <- x_n_3

#if MACRO_LEVEL>=2
	.macro INNER_EDGE_SYMV_NT_L_4_LIBC
#else
	.p2align 4,,15
	FUN_START(inner_edge_symv_nt_l_4_libc)
#endif

	leaq	0(%r12, %r12, 2), %r15

	vmovupd		0(%r13), %ymm12
	vmovupd		0(%r14), %ymm13

	vmovupd		0(%r11), %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm0
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x1, %ymm15, %ymm14, %ymm14
	vfmadd231pd	%ymm14, %ymm6, %ymm13
	
	vmovupd		0(%r11, %r12), %ymm14
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x1, %ymm15, %ymm14, %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm1
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x3, %ymm15, %ymm14, %ymm14
	vfmadd231pd	%ymm14, %ymm7, %ymm13
	
	vmovupd		0(%r11, %r12, 2), %ymm14
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x3, %ymm15, %ymm14, %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm2
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x7, %ymm15, %ymm14, %ymm14
	vfmadd231pd	%ymm14, %ymm8, %ymm13

	vmovupd		0(%r11, %r15), %ymm14
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x7, %ymm15, %ymm14, %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm3
//	vxorpd		%ymm15, %ymm15, %ymm15
//	vblendpd	$ 0x0, %ymm14, %ymm15, %ymm14
//	vfmadd231pd	%ymm14, %ymm9, %ymm13
	
	vmovupd		%ymm13, 0(%r14) 

	addq	$ 32, %r11
	addq	$ 32, %r13
	addq	$ 32, %r14
	
	subq	$ 4, %r10

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_edge_symv_nt_l_4_libc)
#endif





// common inner routine with file scope
//
// input arguments:
// r10   <- kmax
// r11   <- A
// r12   <- bs*sda*sizeof(double) = 32*sda
// r13   <- x_t
// r14   <- z_n
// ymm0  <- [z_t_0a z_t_0b z_t_0c z_t_0d]
// ymm1  <- [z_t_1a z_t_1b z_t_1c z_t_1d]
// ymm2  <- [z_t_2a z_t_2b z_t_2c z_t_2d]
// ymm3  <- [z_t_3a z_t_3b z_t_3c z_t_3d]
// ymm6  <- x_n_0
// ymm7  <- x_n_1
// ymm8  <- x_n_2
// ymm9  <- x_n_3

//
// output arguments:
// r10   <- kmax-4
// r11   <- A+4*k*sizeof(double)
// r12   <- bs*sda*sizeof(double) = 32*sda
// r13   <- x_t+k*sizeof(double)
// r14   <- z_n+k*sizeof(double)
// ymm0  <- [z_t_0a z_t_0b z_t_0c z_t_0d]
// ymm1  <- [z_t_1a z_t_1b z_t_1c z_t_1d]
// ymm2  <- [z_t_2a z_t_2b z_t_2c z_t_2d]
// ymm3  <- [z_t_3a z_t_3b z_t_3c z_t_3d]
// ymm6  <- x_n_0
// ymm7  <- x_n_1
// ymm8  <- x_n_2
// ymm9  <- x_n_3

#if MACRO_LEVEL>=2
	.macro INNER_EDGE_SYMV_NT_U_4_LIBC
#else
	.p2align 4,,15
	FUN_START(inner_edge_symv_nt_u_4_libc)
#endif

	leaq	0(%r12, %r12, 2), %r15

	vmovupd		0(%r13), %ymm12
	vmovupd		0(%r14), %ymm13

	vmovupd		0(%r11), %ymm14
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x1, %ymm14, %ymm15, %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm0
//	vfmadd231pd	%ymm14, %ymm6, %ymm13

	vmovupd	0(%r11, %r12), %ymm14
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x3, %ymm14, %ymm15, %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm1
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x1, %ymm14, %ymm15, %ymm14
	vfmadd231pd	%ymm14, %ymm7, %ymm13
	
	vmovupd	0(%r11, %r12, 2), %ymm14
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x7, %ymm14, %ymm15, %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm2
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x3, %ymm14, %ymm15, %ymm14
	vfmadd231pd	%ymm14, %ymm8, %ymm13

	vmovupd	0(%r11, %r15), %ymm14
//	vxorpd		%ymm15, %ymm15, %ymm15
//	vblendpd	$ 0x7, %ymm14, %ymm15, %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm3
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x7, %ymm14, %ymm15, %ymm14
	vfmadd231pd	%ymm14, %ymm9, %ymm13

	vmovupd		%ymm13, 0(%r14) 

	addq	$ 32, %r11
	addq	$ 32, %r13
	addq	$ 32, %r14
	
	subq	$ 4, %r10

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_edge_symv_nt_u_4_libc)
#endif





//                            1      2              3          4        5          6
// void kernel_dgemv_n_4_libc(int k, double *alpha, double *A, int lda, double *x, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemv_n_4_libc)
	

	PROLOGUE


	// initialize x_n
	movq	ARG2, %r10 // alpha
	vbroadcastsd 0(%r10), %ymm15

	movq	ARG5, %r10 // x_n

	vbroadcastsd 0(%r10), %ymm0
	vmulpd		%ymm15, %ymm0, %ymm0
	vbroadcastsd 8(%r10), %ymm1
	vmulpd		%ymm15, %ymm1, %ymm1
	vbroadcastsd 16(%r10), %ymm2
	vmulpd		%ymm15, %ymm2, %ymm2
	vbroadcastsd 24(%r10), %ymm3
	vmulpd		%ymm15, %ymm3, %ymm3


	// inner edge dsyrk & kernel dgemv nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // lda
	sall	$ 3, %r12d // lda*sizeof(double)
	movq	ARG6, %r13  // z_n

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_N_4_LIBC
#else
	CALL(inner_kernel_gemv_n_4_libc)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemv_n_4_libc)





//                            1      2              3          4        5          6             7         8
// void kernel_dgemv_t_4_libc(int k, double *alpha, double *A, int lda, double *x, double *beta, double *y, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemv_t_4_libc)
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // lda
	sall	$ 3, %r12d // lda*sizeof(double)
//	movslq	%r12d, %r12
	movq	ARG5, %r13  // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_T_4_LIBC
#else
	CALL(inner_kernel_gemv_t_4_libc)
#endif


	// call inner blender t

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11   // beta
	movq	ARG7, %r12 // y 

#if MACRO_LEVEL>=1
	INNER_BLEND_T_LIB4
#else
	CALL(inner_blend_t_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4_LIB4
#else
	CALL(inner_scale_ab_4_lib4)
#endif


	// store

	movq	ARG8, %r10 // z 

#if MACRO_LEVEL>=1
	INNER_STORE_4_LIB4
#else
	CALL(inner_store_4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemv_t_4_lib4)





//                            1      2              3          4        5          6
// void kernel_dsymv_l_4_libc(int k, double *alpha, double *A, int lda, double *x, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsymv_l_4_libc)
	

	PROLOGUE


	// zero accumulation registers y_t

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3

	// initialize x_n
	movq	ARG2, %r10 // alpha
	vbroadcastsd 0(%r10), %ymm15

	movq	ARG5, %r10 // x_n

	vbroadcastsd 0(%r10), %ymm6
	vmulpd		%ymm15, %ymm6, %ymm6
	vbroadcastsd 8(%r10), %ymm7
	vmulpd		%ymm15, %ymm7, %ymm7
	vbroadcastsd 16(%r10), %ymm8
	vmulpd		%ymm15, %ymm8, %ymm8
	vbroadcastsd 24(%r10), %ymm9
	vmulpd		%ymm15, %ymm9, %ymm9


	// inner edge dsyrk & kernel dgemv nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // lda
	sall	$ 3, %r12d // lda*sizeof(double)
	movq	ARG5, %r13  // x_t
	movq	ARG6, %r14  // z_n

#if MACRO_LEVEL>=2
	INNER_EDGE_SYMV_NT_L_4_LIBC
#else
	CALL(inner_edge_symv_nt_l_4_libc)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_NT_4_LIBC
#else
	CALL(inner_kernel_gemv_nt_4_libc)
#endif


	// call inner blend t scale a1

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11   // z_t

#if MACRO_LEVEL>=1
	INNER_BLEND_T_LIB4
#else
	CALL(inner_blend_t_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_SCALE_A1_4_LIB4
#else
	CALL(inner_scale_a1_4_lib4)
#endif


	// store

	movq	ARG6, %r10 // z_t 

#if MACRO_LEVEL>=1
	INNER_STORE_4_LIB4
#else
	CALL(inner_store_4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dsymv_l_4_libc)





//                            1      2              3          4        5            6
// void kernel_dsymv_u_4_libc(int k, double *alpha, double *A, int lda, double *x_t, double *z_n);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsymv_u_4_libc)
	

	PROLOGUE


	// zero accumulation registers y_t

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3

	// initialize x_n
	movq	ARG2, %r10 // alpha
	vbroadcastsd 0(%r10), %ymm15

	movq	ARG5, %r10 // x_n
	leaq	0(%r10, ARG1, 8), %r10 // x_n + k

	vbroadcastsd 0(%r10), %ymm6
	vmulpd		%ymm15, %ymm6, %ymm6
	vbroadcastsd 8(%r10), %ymm7
	vmulpd		%ymm15, %ymm7, %ymm7
	vbroadcastsd 16(%r10), %ymm8
	vmulpd		%ymm15, %ymm8, %ymm8
	vbroadcastsd 24(%r10), %ymm9
	vmulpd		%ymm15, %ymm9, %ymm9


	// inner edge dsyrk & kernel dgemv nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // lda
	sall	$ 3, %r12d // lda*sizeof(double)
	movq	ARG5, %r13  // x_t
	movq	ARG6, %r14  // z_n

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_NT_4_LIBC
#else
	CALL(inner_kernel_gemv_nt_4_libc)
#endif

#if MACRO_LEVEL>=2
	INNER_EDGE_SYMV_NT_U_4_LIBC
#else
	CALL(inner_edge_symv_nt_u_4_libc)
#endif


	// call inner blend t scale a1

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11   // z_t
	leaq	0(%r11, ARG1, 8), %r11 // z_t + k

#if MACRO_LEVEL>=1
	INNER_BLEND_T_LIB4
#else
	CALL(inner_blend_t_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_SCALE_A1_4_LIB4
#else
	CALL(inner_scale_a1_4_lib4)
#endif


	// store

	movq	ARG6, %r10 // z_t 
	leaq	0(%r10, ARG1, 8), %r10 // z_t + k

#if MACRO_LEVEL>=1
	INNER_STORE_4_LIB4
#else
	CALL(inner_store_4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dsymv_u_4_libc)





