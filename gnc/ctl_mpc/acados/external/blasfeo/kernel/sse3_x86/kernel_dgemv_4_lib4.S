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
// xmm0  <- [z0 z1]_a
// xmm1  <- [z2 z3]_b
// xmm2  <- [z0 z1]_a
// xmm3  <- [z2 z3]_b

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
	
	movddup		0(%ecx), %xmm5
	movapd		0(%ebx), %xmm4
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm0
	movapd		16(%ebx), %xmm4
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1

	movddup		8(%ecx), %xmm5
	movapd		32(%ebx), %xmm4
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm2
	movapd		48(%ebx), %xmm4
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm3

	movddup		16(%ecx), %xmm5
	movapd		64(%ebx), %xmm4
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm0
	movapd		80(%ebx), %xmm4
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1

	movddup		24(%ecx), %xmm5
	movapd		96(%ebx), %xmm4
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm2
	movapd		112(%ebx), %xmm4
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm3

	subl	$ 4, %eax
	addl	$ 128, %ebx
	addl	$ 32, %ecx
	
	cmpl	$ 3, %eax
	jg		1b // main loop 


	// consider clean-up
	cmpl	$ 0, %eax
	jle		2f // return

0: // clean-up
	
	movddup		0(%ecx), %xmm5
	movapd		0(%ebx), %xmm4
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm0
	movapd		16(%ebx), %xmm4
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1

	subl	$ 1, %eax
	addl	$ 32, %ebx
	addl	$ 8, %ecx

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
// xmm0  <- [z0a z0b]
// xmm1  <- [z2a z2b]
// xmm2  <- [z1a z1b]
// xmm3  <- [z3a z3b]

//
// output arguments:

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
	prefetcht0	64(%ebx, %ecx, 1) // software prefetch

	movupd		0(%edx), %xmm4

	movapd		0(%ebx), %xmm5
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0

	movapd		32(%ebx), %xmm5
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm2

	movapd		64(%ebx), %xmm5
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm1

	movapd		96(%ebx), %xmm5
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3

	movupd		16(%edx), %xmm4

	movapd		16(%ebx), %xmm5
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0

	movapd		48(%ebx), %xmm5
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm2

	movapd		80(%ebx), %xmm5
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm1

	movapd		112(%ebx), %xmm5
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3

	subl	$ 4, %eax
	addl	%ecx, %ebx
	addl	$ 32, %edx
	
	cmpl	$ 3, %eax
	jg		1b // main loop 


	// consider clean-up
	cmpl	$ 0, %eax
	jle		2f // return

0: // clean-up
	
	movsd		0(%edx), %xmm4

	movsd		0(%ebx), %xmm5
	mulsd		%xmm4, %xmm5
	addsd		%xmm5, %xmm0

	movsd		32(%ebx), %xmm5
	mulsd		%xmm4, %xmm5
	addsd		%xmm5, %xmm2

	movsd		64(%ebx), %xmm5
	mulsd		%xmm4, %xmm5
	addsd		%xmm5, %xmm1

	movsd		96(%ebx), %xmm5
	mulsd		%xmm4, %xmm5
	addsd		%xmm5, %xmm3
	
	subl	$ 1, %eax
	addl	$ 8, %ebx
	addl	$ 8, %edx
	
	cmpl	$ 0, %eax
	jg		0b // main loop 

	
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
// rsi   <- offA
// xmm0  <- [z0a z0b]
// xmm1  <- [z1a z1b]
// xmm2  <- [z2a z2b]
// xmm3  <- [z3a z3b]

//
// output arguments:
// eax   <- 
// ebx   <- 
// ecx   <- 
// edx   <- 
// esi   <- offA
// xmm0  <- [z0a z0b]
// xmm1  <- [z1a z1b]
// xmm2  <- [z2a z2b]
// xmm3  <- [z3a z3b]

#if MACRO_LEVEL>=2
	.macro INNER_EDGE_GEMV_ADD_T_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_gemv_add_t_4_lib4)
#endif

	cmpl			$ 0, %esi				// offset==0
	jle				2f						// end

	cmpl			$ 0, %eax				// k==0
	jle				2f						// end

	movl			$ 4, %edi				// load 4
	subl			%esi, %edi			// 4-offsetA
	cmpl			%eax, %edi			// k > 4-offsetA
	cmovgl			%eax, %edi			// kend=min(k,4-offsetA)

//	movl			%esi, %ebp				// load offsetA
//	sall			$ 3, %ebp				// offsetA*sizeof(double)
//	addl			%ebp, %ebx				// A+offsetA*sizeof(double)

1:
	movsd	0(%edx), %xmm7

	movsd	0(%ebx), %xmm6
	mulsd	%xmm7, %xmm6
	addsd	%xmm6, %xmm0
	subl	$ 1, %eax

	movsd	32(%ebx), %xmm6
	mulsd	%xmm7, %xmm6
	addsd	%xmm6, %xmm1

	movsd	64(%ebx), %xmm6
	mulsd	%xmm7, %xmm6
	addsd	%xmm6, %xmm2

	movsd	96(%ebx), %xmm6
	mulsd	%xmm7, %xmm6
	addsd	%xmm6, %xmm3

	subl	$ 1, %eax				// k=-1
	subl	$ 1, %edi				// k_panel=-1
	addl	$ 8, %ebx				// A=+bs
	addl	$ 8, %edx				// x=+1

	cmpl	$ 0, %edi				// if k_panel=0
	jg		1b						// loop 1

	cmpl	$ 0, %eax				// if k=0
	jle		2f						// end

	addl	%ecx, %ebx				// B=Boff+sdb*bs
	subl	$ 32, %ebx				// B-=4*sizeof(double) (loop+offsetB)

2:

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
// xmm0  <- [z0 z1]_a
// xmm1  <- [z2 z3]_b
// xmm2  <- [z0 z1]_a
// xmm3  <- [z2 z3]_b
//
// output arguments:
// eax  <- alpha
// ebx  <- beta
// ecx  <- y
// xmm0 <- [z0 z1]
// xmm1 <- [z2 z3]

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_N_SCALE_AB_4_LIB4
#else
	.align 16
	.type inner_blend_n_scale_ab_4_lib4, @function
inner_blend_n_scale_ab_4_lib4:
#endif

	// reduction
	addpd		%xmm2, %xmm0
	addpd		%xmm3, %xmm1

	// alpha
	movddup		0(%eax), %xmm7
	mulpd		%xmm7, %xmm0
	mulpd		%xmm7, %xmm1

	// beta
	movddup		0(%ebx), %xmm7
	movupd		0(%ecx), %xmm6
	mulpd		%xmm7, %xmm6
	addpd		%xmm6, %xmm0
	movupd		16(%ecx), %xmm6
	mulpd		%xmm7, %xmm6
	addpd		%xmm6, %xmm1

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
// xmm0  <- [z0a z0b]
// xmm1  <- [z2a z2b]
// xmm2  <- [z1a z1b]
// xmm3  <- [z3a z3b]
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
	haddpd		%xmm2, %xmm0
	haddpd		%xmm3, %xmm1

	// alpha
	movddup		0(%eax), %xmm7
	mulpd		%xmm7, %xmm0
	mulpd		%xmm7, %xmm1

	// beta
	movddup		0(%ebx), %xmm7
	movupd		0(%ecx), %xmm6
	mulpd		%xmm7, %xmm6
	addpd		%xmm6, %xmm0
	movupd		16(%ecx), %xmm6
	mulpd		%xmm7, %xmm6
	addpd		%xmm6, %xmm1
	
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
// xmm0 <- [z0 z1]
// xmm1 <- [z2 z3]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4_LIB4
#else
	.align 16
	.type inner_store_4_lib4, @function
inner_store_4_lib4:
#endif
	
	movupd		%xmm0,  0(%eax)
	movupd		%xmm1, 16(%eax)
	
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
	
	cmpl	$ 0, %ebx
	jle		0f // return

	movsd 	%xmm0, 0(%eax)

	cmpl	$ 1, %ebx
	jle		0f // return

	movhpd 	%xmm0, 8(%eax)

	cmpl	$ 2, %ebx
	jle		0f // return

	movsd 	%xmm1, 16(%eax)

	cmpl	$ 3, %ebx
	jle		0f // return

	movhpd 	%xmm1, 24(%eax)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_store_4_vs_lib4, .-inner_store_4_vs_lib4
#endif
#endif





//                            1      2              3          4          5             6          7
// void kernel_dgemv_n_4_lib4(int k, double *alpha, double *A, double *x, double *beta, double *y, double *z);

	.align 16
	.globl kernel_dgemv_n_4_lib4
	.type kernel_dgemv_n_4_lib4, @function
kernel_dgemv_n_4_lib4:
	
	PROLOGUE

	// zero accumulation registers

	xorpd	%xmm0, %xmm0
	movapd	%xmm0, %xmm1
	movapd	%xmm0, %xmm2
	movapd	%xmm0, %xmm3


	// call inner dgemv kernel n

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

	.size	kernel_dgemv_n_4_lib4, .-kernel_dgemv_n_4_lib4






//                               1      2              3          4          5             6          7          8
// void kernel_dgemv_n_4_vs_lib4(int k, double *alpha, double *A, double *x, double *beta, double *y, double *z, int k1);

	.align 16
	.globl kernel_dgemv_n_4_vs_lib4
	.type kernel_dgemv_n_4_vs_lib4, @function
kernel_dgemv_n_4_vs_lib4:
	
	PROLOGUE

	// zero accumulation registers

	xorpd	%xmm0, %xmm0
	movapd	%xmm0, %xmm1
	movapd	%xmm0, %xmm2
	movapd	%xmm0, %xmm3


	// call inner dgemv kernel n

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

	.size	kernel_dgemv_n_4_vs_lib4, .-kernel_dgemv_n_4_vs_lib4






//                            1      2              3          4        5          6             7         8          9
// void kernel_dgemv_t_4_lib4(int k, double *alpha, int offA, double *A, int sda, double *x, double *beta, double *y, double *z);

	.align 16
	.globl kernel_dgemv_t_4_lib4
	.type kernel_dgemv_t_4_lib4, @function
kernel_dgemv_t_4_lib4:
	
	PROLOGUE

	// zero accumulation registers

	xorpd	%xmm0, %xmm0
	movapd	%xmm0, %xmm1
	movapd	%xmm0, %xmm2
	movapd	%xmm0, %xmm3


	// call inner dgemv kernel n

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

	.size	kernel_dgemv_t_4_lib4, .-kernel_dgemv_t_4_lib4





//                               1      2              3          4        5          6             7         8           9         10
// void kernel_dgemv_t_4_vs_lib4(int k, double *alpha, int offA, double *A, int sda, double *x, double *beta, double *y, double *z, int k1);

	.align 16
	.globl kernel_dgemv_t_4_vs_lib4
	.type kernel_dgemv_t_4_vs_lib4, @function
kernel_dgemv_t_4_vs_lib4:
	
	PROLOGUE

	// zero accumulation registers

	xorpd	%xmm0, %xmm0
	movapd	%xmm0, %xmm1
	movapd	%xmm0, %xmm2
	movapd	%xmm0, %xmm3


	// call inner dgemv kernel n

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

	.size	kernel_dgemv_t_4_vs_lib4, .-kernel_dgemv_t_4_vs_lib4





	.section	.note.GNU-stack,"",@progbits

