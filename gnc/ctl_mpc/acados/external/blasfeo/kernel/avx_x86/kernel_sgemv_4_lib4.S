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

#if defined(OS_LINUX)

#define STACKSIZE 16
#define ARG1  STACKSIZE +  4(%esp)
#define ARG2  STACKSIZE +  8(%esp)
#define ARG3  STACKSIZE + 12(%esp)
#define ARG4  STACKSIZE + 16(%esp)
#define ARG5  STACKSIZE + 20(%esp)
#define ARG6  STACKSIZE + 24(%esp)
#define ARG7  STACKSIZE + 28(%esp)
#define ARG8  STACKSIZE + 32(%esp)
#define ARG9  STACKSIZE + 36(%esp)
#define ARG10 STACKSIZE + 40(%esp)

#if 1

#define PROLOGUE \
	subl	$ 16, %esp; \
	movl	%ebx, 0(%esp); \
	movl	%esi, 4(%esp); \
	movl	%edi, 8(%esp); \
	movl	%ebp, 12(%esp);
#define EPILOGUE \
	movl	0(%esp), %ebx; \
	movl	4(%esp), %esi; \
	movl	8(%esp), %edi; \
	movl	12(%esp), %ebp; \
	addl	$ 16, %esp;

#else

#define PROLOGUE \
	pushl	%ebp; \
	pushl	%edi; \
	pushl	%esi; \
	pushl	%ebx;
#define EPILOGUE \
	popl	%ebx; \
	popl	%esi; \
	popl	%edi; \
	popl	%ebp;

#endif

#define GLOB_FUN_START(NAME) \
	.globl NAME; \
	.type NAME, @function; \
NAME:
#define FUN_START(NAME) \
	.type NAME, @function; \
NAME:
#define FUN_END(NAME) \
	.size	NAME, .-NAME
#define CALL(NAME) \
	call NAME
//#define ZERO_ACC \
//	vxorpd	%ymm0, %ymm0, %ymm0; \
//	vmovapd	%ymm0, %ymm1; \
//	vmovapd	%ymm0, %ymm2; \
//	vmovapd	%ymm0, %ymm3
//#define NEG_ACC \
//	vmovapd		.LC11(%rip), %ymm15; \
//	vxorpd		%ymm15, %ymm0, %ymm0; \
//	vxorpd		%ymm15, %ymm1, %ymm1; \
//	vxorpd		%ymm15, %ymm2, %ymm2; \
//	vxorpd		%ymm15, %ymm3, %ymm3

#else

#error wrong OS

#endif



	.text



// common inner routine with file scope
//
// input arguments:
// eax   <- k
// ebx   <- A
// ecx   <- x
// xmm0  <- [z0 z1 z2 z3]_a
// xmm1  <- [z0 z1 z2 z3]_b
// xmm2  <- [z0 z1 z2 z3]_c
// xmm3  <- [z0 z1 z2 z3]_d

//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMV_ADD_N_4_LIB4
#else
	.align 16
	.type inner_kernel_gemv_add_n_4_lib4, @function
inner_kernel_gemv_add_n_4_lib4:
#endif
	
	cmpl	$ 0, %eax
	jle		2f // return

	cmpl	$ 4, %eax
	jl		0f // clean-up loop

	// main loop
	.align 8
1: // main loop
	
	vmovaps	0(%ebx), %xmm4
	vbroadcastss	0(%ecx), %xmm5
	vmulps	%xmm4, %xmm5, %xmm7
	vaddps	%xmm0, %xmm7, %xmm0
	
	vmovaps	16(%ebx), %xmm4
	vbroadcastss	4(%ecx), %xmm5
	vmulps	%xmm4, %xmm5, %xmm7
	vaddps	%xmm1, %xmm7, %xmm1
	
	vmovaps	32(%ebx), %xmm4
	vbroadcastss	8(%ecx), %xmm5
	vmulps	%xmm4, %xmm5, %xmm7
	vaddps	%xmm2, %xmm7, %xmm2

	vmovaps	48(%ebx), %xmm4
	vbroadcastss	12(%ecx), %xmm5
	vmulps	%xmm4, %xmm5, %xmm7
	vaddps	%xmm3, %xmm7, %xmm3
	
	subl	$ 4, %eax
	addl	$ 64, %ebx
	addl	$ 16, %ecx
	
	cmpl	$ 3, %eax

	jg		1b // main loop 


	// consider clean-up
	cmpl	$ 0, %eax
	jle		2f // return

0: // clean-up
	
	vmovaps	0(%ebx), %xmm4
	vbroadcastss	0(%ecx), %xmm5
	vmulps	%xmm4, %xmm5, %xmm7
	vaddps	%xmm0, %xmm7, %xmm0
	
	addl	$ 16, %ebx
	addl	$ 4, %ecx
	
	subl	$ 1, %eax
	cmpl	$ 0, %eax

	jg		0b // clean

2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	.size	inner_kernel_gemv_add_n_4_lib4, .-inner_kernel_gemv_add_n_4_lib4
#endif





// common inner routine with file scope
//
// input arguments:
// eax  <- k
// ebx   <- A
// ecx   <- bs*sda*sizeof(double) = 32*sda
// edx   <- x
// xmm0  <- [z0a z0b z0c z0d]
// xmm1  <- [z1a z1b z1c z1d]
// xmm2  <- [z2a z2b z2c z2d]
// xmm3  <- [z3a z3b z3c z3d]

//
// output arguments:
// eax  <- 0
// ebx   <- A+4*k*sizeof(double)
// ecx   <- bs*sda*sizeof(double) = 32*sda
// edx   <- x+k*sizeof(double)
// xmm0  <- [z0a z0b z0c z0d]
// xmm1  <- [z1a z1b z1c z1d]
// xmm2  <- [z2a z2b z2c z2d]
// xmm3  <- [z3a z3b z3c z3d]

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMV_ADD_T_4_LIB4
#else
	.align 16
	.type inner_kernel_gemv_add_t_4_lib4, @function
inner_kernel_gemv_add_t_4_lib4:
#endif

	cmpl	$ 0, %eax
	jle		2f // return

	cmpl	$ 4, %eax
	jl		0f // clean-up loop

	// main loop
	.align 8
1: // main loop
	
	prefetcht0	0(%ebx, %ecx, 1) // software prefetch

	vmovups	0(%edx), %xmm4

	vmovaps	0(%ebx), %xmm5
	vmulps	%xmm5, %xmm4, %xmm7
	vaddps	%xmm0, %xmm7, %xmm0
	
	subl	$ 4, %eax

	vmovaps	16(%ebx), %xmm5
	vmulps	%xmm5, %xmm4, %xmm7
	vaddps	%xmm1, %xmm7, %xmm1
	
	vmovaps	32(%ebx), %xmm5
	vmulps	%xmm5, %xmm4, %xmm7
	vaddps	%xmm2, %xmm7, %xmm2

	vmovaps	48(%ebx), %xmm5
	vmulps	%xmm5, %xmm4, %xmm7
	vaddps	%xmm3, %xmm7, %xmm3
	
	addl	%ecx, %ebx
	addl	$ 16, %edx
	
	cmpl	$ 3, %eax

	jg		1b // main loop 


	// consider clean-up
	cmpl	$ 0, %eax
	jle		2f // return

0: // clean-up
	
	vcvtsi2ss	%eax, %xmm6, %xmm6
	vmovups		.LC00, %xmm5
	vshufps		$ 0x00, %xmm6, %xmm6, %xmm6
	vsubps		%xmm6, %xmm5, %xmm6

	vmaskmovps	0(%edx), %xmm6, %xmm4

	vmaskmovps	0(%ebx), %xmm6, %xmm5
	vmulps	%xmm5, %xmm4, %xmm7
	vaddps	%xmm0, %xmm7, %xmm0
	
	vmaskmovps	16(%ebx), %xmm6, %xmm5
	vmulps	%xmm5, %xmm4, %xmm7
	vaddps	%xmm1, %xmm7, %xmm1
	
	vmaskmovps	32(%ebx), %xmm6, %xmm5
	vmulps	%xmm5, %xmm4, %xmm7
	vaddps	%xmm2, %xmm7, %xmm2

	vmaskmovps	48(%ebx), %xmm6, %xmm5
	vmulps	%xmm5, %xmm4, %xmm7
	vaddps	%xmm3, %xmm7, %xmm3

	sall	$ 3, %eax
	addl	%eax, %ebx
	addl	%eax, %edx
	xorl	%eax, %eax
	
	
2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_gemv_add_t_4_lib4, .-inner_kernel_gemv_add_t_4_lib4
#endif
#endif





// common inner routine with file scope
//
// input arguments:
// eax   <- k
// ebx   <- A
// ecx   <- bs*sda*sizeof(double) = 32*sda
// edx   <- x
// esi   <- offA
// ymm0  <- [z0a z0b z0c z0d]
// ymm1  <- [z1a z1b z1c z1d]
// ymm2  <- [z2a z2b z2c z2d]
// ymm3  <- [z3a z3b z3c z3d]

//
// output arguments:
// eax   <- 
// ebx   <- 
// ecx   <- 
// edx   <- 
// esi   <- offA
// ymm0  <- [z0a z0b z0c z0d]
// ymm1  <- [z1a z1b z1c z1d]
// ymm2  <- [z2a z2b z2c z2d]
// ymm3  <- [z3a z3b z3c z3d]

#if MACRO_LEVEL>=2
	.macro INNER_EDGE_GEMV_ADD_T_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_gemv_add_t_4_lib4)
#endif

	cmpl	$ 0, %esi
	jle		0f // return

	movl	%esi, %edi
	sall	$ 2, %edi // offA*sizeof(float)

	subl	%edi, %ebx // A - offA
	subl	%edi, %edx // x - offA

	movl	%eax, %edi // kmax
	addl	%esi, %edi // kmax + offA

	vcvtsi2ss	%esi, %xmm6, %xmm6 // offA
	vcvtsi2ss	%edi, %xmm7, %xmm7 // offA + kmax
	vmovupd		.LC00, %ymm5
	vshufps		$ 0x00, %xmm6, %xmm6, %xmm6
	vshufps		$ 0x00, %xmm7, %xmm7, %xmm7
	vsubpd		%xmm5, %xmm6, %xmm6
	vsubpd		%xmm7, %xmm5, %xmm7
	vandpd		%xmm7, %xmm6, %xmm6

	vmaskmovps	0(%edx), %xmm6, %xmm4

	vmaskmovps	0(%ebx), %xmm6, %xmm5
	vmulps	%xmm5, %xmm4, %xmm7
	vaddps	%xmm0, %xmm7, %xmm0
	
	vmaskmovps	16(%ebx), %xmm6, %xmm5
	vmulps	%xmm5, %xmm4, %xmm7
	vaddps	%xmm1, %xmm7, %xmm1
	
	vmaskmovps	32(%ebx), %xmm6, %xmm5
	vmulps	%xmm5, %xmm4, %xmm7
	vaddps	%xmm2, %xmm7, %xmm2

	vmaskmovps	48(%ebx), %xmm6, %xmm5
	vmulps	%xmm5, %xmm4, %xmm7
	vaddps	%xmm3, %xmm7, %xmm3

	addl	$ 16, %edx // x + 4
	addl	%ecx, %ebx // A + bs*sda
		
	addl	%esi, %eax
	subl	$ 4, %eax // kmax - (4-offA)
	
0: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_edge_gemv_add_t_4_lib4)
#endif





// common inner routine with file scope
//
// blend for ta==n, scale for generic alpha and beta
//
// input arguments:
// eax  <- alpha
// ebx  <- beta
// ecx  <- y
// xmm0 <- [z0 z1 z2 z3]_a
// xmm1 <- [z0 z1 z2 z3]_b
// xmm2 <- [z0 z1 z2 z3]_c
// xmm3 <- [z0 z1 z2 z3]_d
//
// output arguments:
// eax  <- alpha
// ebx  <- beta
// ecx  <- y
// xmm0 <- [z0 z1 z2 z3]

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_N_SCALE_AB_4_LIB4
#else
	.align 16
	.type inner_blend_n_scale_ab_4_lib4, @function
inner_blend_n_scale_ab_4_lib4:
#endif

	// reduction
	vaddps	%xmm0, %xmm1, %xmm0
	vaddps	%xmm2, %xmm3, %xmm2
	vaddps	%xmm0, %xmm2, %xmm0

	// alpha
	vbroadcastss	0(%eax), %xmm7
	vmulps	%xmm0, %xmm7, %xmm0

	// beta
	vbroadcastss	0(%ebx), %xmm7
	vmovups		0(%ecx), %xmm6
	vmulps		%xmm7, %xmm6, %xmm6
	vaddps		%xmm0, %xmm6, %xmm0

#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	.size	inner_blend_n_scale_ab_4_lib4, .-inner_blend_n_scale_ab_4_lib4
#endif





// common inner routine with file scope
//
// blend for ta==t, scale for generic alpha and beta
//
// input arguments:
// eax  <- alpha
// ebx  <- beta
// ecx  <- y
// xmm0 <- [z0a z0b z0c z0d]
// xmm1 <- [z1a z1b z1c z1d]
// xmm2 <- [z2a z2b z2c z2d]
// xmm3 <- [z3a z3b z3c z3d]
//
// output arguments:
// eax  <- alpha
// ebx  <- beta
// ecx  <- y
// xmm0 <- [z0 z1 z2 z3]

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_T_SCALE_AB_4_LIB4
#else
	.align 16
	.type inner_blend_t_scale_ab_4_lib4, @function
inner_blend_t_scale_ab_4_lib4:
#endif

	// reduction
	vhaddps	%xmm1, %xmm0, %xmm0
	vhaddps	%xmm3, %xmm2, %xmm2
	vhaddps	%xmm2, %xmm0, %xmm0

	// alpha
	vbroadcastss	0(%eax), %xmm7
	vmulps	%xmm0, %xmm7, %xmm0

	// beta
	vbroadcastss	0(%ebx), %xmm7
	vmovups		0(%ecx), %xmm6
	vmulps		%xmm7, %xmm6, %xmm6
	vaddps		%xmm0, %xmm6, %xmm0
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	.size	inner_blend_t_scale_ab_4_lib4, .-inner_blend_t_scale_ab_4_lib4
#endif





// common inner routine with file scope
//
// store 
//
// input arguments:
// eax  <- z
// xmm0 <- [z0 z1 z2 z3]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4_LIB4
#else
	.align 16
	.type inner_store_4_lib4, @function
inner_store_4_lib4:
#endif
	
	vmovups %xmm0,  0(%eax)
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_store_4_lib4, .-inner_store_4_lib4
#endif





// common inner routine with file scope
//
// store vs
//
// input arguments:
// eax   <- D
// ebx   <- km
// xmm0  <- [z0 z1 z2 z3]
//
// output arguments:
// eax   <- D
// ebx   <- km
// xmm0  <- [z0 z1 z2 z3]

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4_VS_LIB4
#else
	.align 16
	.type inner_store_4_vs_lib4, @function
inner_store_4_vs_lib4:
#endif
	
	vcvtsi2ss	%ebx, %xmm7, %xmm7
	vmovups		.LC00, %xmm6
	vshufps		$ 0x00, %xmm7, %xmm7, %xmm7
	vsubps		%xmm7, %xmm6, %xmm7

	vmaskmovps	%xmm0, %xmm7,  0(%eax)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_store_4_vs_lib4, .-inner_store_4_vs_lib4
#endif
#endif





//                            1      2              3          4          5             6          7
// void kernel_sgemv_n_4_lib4(int k, double *alpha, double *A, double *x, double *beta, double *y, double *z);

	.align 16
	.globl kernel_sgemv_n_4_lib4
	.type kernel_sgemv_n_4_lib4, @function
kernel_sgemv_n_4_lib4:
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%xmm0, %xmm0, %xmm0
	vmovaps	%xmm0, %xmm1
	vmovaps	%xmm0, %xmm2
	vmovaps	%xmm0, %xmm3


	// call inner sgemv kernel n

	movl	ARG1, %eax // k
	movl	ARG3, %ebx  // A
	movl	ARG4, %ecx  // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_N_4_LIB4
#else
	call inner_kernel_gemv_add_n_4_lib4
#endif


	// call inner blend n scale ab

	movl	ARG2, %eax // alpha
	movl	ARG5, %ebx   // beta
	movl	ARG6, %ecx   // y

#if MACRO_LEVEL>=1
	INNER_BLEND_N_SCALE_AB_4_LIB4
#else
	call inner_blend_n_scale_ab_4_lib4
#endif


	// store

	movl	ARG7, %eax // z 

#if MACRO_LEVEL>=1
	INNER_STORE_4_LIB4
#else
	call inner_store_4_lib4
#endif


	EPILOGUE

	ret

	.size	kernel_sgemv_n_4_lib4, .-kernel_sgemv_n_4_lib4






//                               1      2              3          4          5             6          7          8
// void kernel_sgemv_n_4_vs_lib4(int k, double *alpha, double *A, double *x, double *beta, double *y, double *z, int k1);

	.align 16
	.globl kernel_sgemv_n_4_vs_lib4
	.type kernel_sgemv_n_4_vs_lib4, @function
kernel_sgemv_n_4_vs_lib4:
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%xmm0, %xmm0, %xmm0
	vmovaps	%xmm0, %xmm1
	vmovaps	%xmm0, %xmm2
	vmovaps	%xmm0, %xmm3


	// call inner sgemv kernel n

	movl	ARG1, %eax // k
	movl	ARG3, %ebx  // A
	movl	ARG4, %ecx  // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_N_4_LIB4
#else
	call inner_kernel_gemv_add_n_4_lib4
#endif


	// call inner blend n scale ab

	movl	ARG2, %eax // alpha
	movl	ARG5, %ebx   // beta
	movl	ARG6, %ecx   // y

#if MACRO_LEVEL>=1
	INNER_BLEND_N_SCALE_AB_4_LIB4
#else
	call inner_blend_n_scale_ab_4_lib4
#endif


	// store

	movl	ARG7, %eax // z 
	movl	ARG8, %ebx // k1 

#if MACRO_LEVEL>=1
	INNER_STORE_4_VS_LIB4
#else
	call inner_store_4_vs_lib4
#endif


	EPILOGUE

	ret

	.size	kernel_sgemv_n_4_vs_lib4, .-kernel_sgemv_n_4_vs_lib4






//                            1      2              3          4        5          6             7         8          9
// void kernel_sgemv_t_4_lib4(int k, double *alpha, int offA, double *A, int sda, double *x, double *beta, double *y, double *z);

	.align 16
	.globl kernel_sgemv_t_4_lib4
	.type kernel_sgemv_t_4_lib4, @function
kernel_sgemv_t_4_lib4:
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%xmm0, %xmm0, %xmm0
	vmovaps	%xmm0, %xmm1
	vmovaps	%xmm0, %xmm2
	vmovaps	%xmm0, %xmm3


	// call inner sgemv kernel n

	movl	ARG1, %eax // k
	movl	ARG4, %ebx  // A
	movl	ARG5, %ecx // sda
	sall	$ 5, %ecx // 4*sda*sizeof(double)
	movl	ARG6, %edx  // x
	movl	ARG3, %esi  // offA

#if MACRO_LEVEL>=2
	INNER_EDGE_GEMV_ADD_T_4_LIB4
#else
	CALL(inner_edge_gemv_add_t_4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_T_4_LIB4
#else
	call inner_kernel_gemv_add_t_4_lib4
#endif


	// call inner blender t

	movl	ARG2, %eax // alpha
	movl	ARG7, %ebx   // beta
	movl	ARG8, %ecx // y 

#if MACRO_LEVEL>=1
	INNER_BLEND_T_SCALE_AB_4_LIB4
#else
	call inner_blend_t_scale_ab_4_lib4
#endif


	// store

	movl	ARG9, %eax // z 

#if MACRO_LEVEL>=1
	INNER_STORE_4_LIB4
#else
	call inner_store_4_lib4
#endif


	EPILOGUE

	ret

	.size	kernel_sgemv_t_4_lib4, .-kernel_sgemv_t_4_lib4





//                               1      2              3          4        5          6             7         8           9     10
// void kernel_sgemv_t_4_vs_lib4(int k, double *alpha, offA, double *A, int sda, double *x, double *beta, double *y, double *z, int k1);

	.align 16
	.globl kernel_sgemv_t_4_vs_lib4
	.type kernel_sgemv_t_4_vs_lib4, @function
kernel_sgemv_t_4_vs_lib4:
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%xmm0, %xmm0, %xmm0
	vmovaps	%xmm0, %xmm1
	vmovaps	%xmm0, %xmm2
	vmovaps	%xmm0, %xmm3


	// call inner sgemv kernel n

	movl	ARG1, %eax // k
	movl	ARG4, %ebx  // A
	movl	ARG5, %ecx // sda
	sall	$ 5, %ecx // 4*sda*sizeof(double)
	movl	ARG6, %edx  // x
	movl	ARG3, %esi  // offA

#if MACRO_LEVEL>=2
	INNER_EDGE_GEMV_ADD_T_4_LIB4
#else
	CALL(inner_edge_gemv_add_t_4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_T_4_LIB4
#else
	call inner_kernel_gemv_add_t_4_lib4
#endif


	// call inner blender t

	movl	ARG2, %eax // alpha
	movl	ARG7, %ebx   // beta
	movl	ARG8, %ecx // y 

#if MACRO_LEVEL>=1
	INNER_BLEND_T_SCALE_AB_4_LIB4
#else
	call inner_blend_t_scale_ab_4_lib4
#endif


	// store

	movl	ARG9, %eax // z 
	movl	ARG10, %ebx // k1 

#if MACRO_LEVEL>=1
	INNER_STORE_4_VS_LIB4
#else
	call inner_store_4_vs_lib4
#endif


	EPILOGUE

	ret

	.size	kernel_sgemv_t_4_vs_lib4, .-kernel_sgemv_t_4_vs_lib4





	// read-only data
	.section	.rodata.cst32,"aM",@progbits,32
	.align 16
.LC00: // { 3.5 2.5 1.5 0.5 }
	.long	1056964608
	.long	1069547520
	.long	1075838976
	.long	1080033280

	.section	.note.GNU-stack,"",@progbits

