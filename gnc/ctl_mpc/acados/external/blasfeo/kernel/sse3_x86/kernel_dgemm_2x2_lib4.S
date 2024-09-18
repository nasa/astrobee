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
#define ARG11 STACKSIZE + 44(%esp)

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

#else

#error wrong OS

#endif



	.text



// common inner routine with file scope
//
// input arguments:
// eax   <- k
// ebx   <- A
// ecx   <- B
// xmm0 <- [d00 d10]
// xmm1 <- [d01 d11]

//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NT_2X2_LIB4
#else
	.align 16
	.type inner_kernel_gemm_add_nt_2x2_lib4, @function
inner_kernel_gemm_add_nt_2x2_lib4:
#endif

	cmpl	$ 0, %eax
	jle		2f // return

	// preload

	cmpl	$ 4, %eax
	jle		0f // consider clean-up loop

	// main loop
	.align 8
1: // main loop

	// unroll 0
	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		8(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1

	// unroll 1
	movddup		32(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		32(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		40(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1

	// unroll 2
	movddup		64(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		64(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		72(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1

	// unroll 3
	movddup		96(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		96(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		104(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1


	subl	$ 4, %eax
	addl	$ 128, %ecx
	addl	$ 128, %ebx

	cmpl	$ 4, %eax
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %eax
	jle		4f // clean1

	// unroll 0
	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		8(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1

	// unroll 1
	movddup		32(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		32(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		40(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1

	// unroll 2
	movddup		64(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		64(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		72(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1

	// unroll 3
	movddup		96(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		96(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		104(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1

	subl	$ 4, %eax
	addl	$ 128, %ecx
	addl	$ 128, %ebx

//	cmpl	$ 3, %eax
	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %eax
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		8(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1

	subl	$ 1, %eax
	addl	$ 32, %ecx
	addl	$ 32, %ebx

	cmpl	$ 0, %eax
	jg		3b // clean up loop

2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	.size	inner_kernel_gemm_add_nt_2x2_lib4, .-inner_kernel_gemm_add_nt_2x2_lib4
#endif





// common inner routine with file scope
//
// input arguments:
// eax   <- k
// ebx   <- A
// ecx   <- B
// xmm0 <- [d00 d10]
// xmm1 <- [d01 d11]

//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_SUB_NT_2X2_LIB4
#else
	.align 16
	.type inner_kernel_gemm_sub_nt_2x2_lib4, @function
inner_kernel_gemm_sub_nt_2x2_lib4:
#endif

	cmpl	$ 0, %eax
	jle		2f // return

	// preload

	cmpl	$ 4, %eax
	jle		0f // consider clean-up loop

	// main loop
	.align 8
1: // main loop

	// unroll 0
	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	subpd		%xmm5, %xmm0
	movddup		8(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	subpd		%xmm4, %xmm1

	// unroll 1
	movddup		32(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		32(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	subpd		%xmm5, %xmm0
	movddup		40(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	subpd		%xmm4, %xmm1

	// unroll 2
	movddup		64(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		64(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	subpd		%xmm5, %xmm0
	movddup		72(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	subpd		%xmm4, %xmm1

	// unroll 3
	movddup		96(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		96(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	subpd		%xmm5, %xmm0
	movddup		104(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	subpd		%xmm4, %xmm1


	subl	$ 4, %eax
	addl	$ 128, %ecx
	addl	$ 128, %ebx

	cmpl	$ 4, %eax
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %eax
	jle		4f // clean1

	// unroll 0
	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	subpd		%xmm5, %xmm0
	movddup		8(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	subpd		%xmm4, %xmm1

	// unroll 1
	movddup		32(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		32(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	subpd		%xmm5, %xmm0
	movddup		40(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	subpd		%xmm4, %xmm1

	// unroll 2
	movddup		64(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		64(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	subpd		%xmm5, %xmm0
	movddup		72(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	subpd		%xmm4, %xmm1

	// unroll 3
	movddup		96(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		96(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	subpd		%xmm5, %xmm0
	movddup		104(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	subpd		%xmm4, %xmm1

	subl	$ 4, %eax
	addl	$ 128, %ecx
	addl	$ 128, %ebx

//	cmpl	$ 3, %eax
	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %eax
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	subpd		%xmm5, %xmm0
	movddup		8(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	subpd		%xmm4, %xmm1

	subl	$ 1, %eax
	addl	$ 32, %ecx
	addl	$ 32, %ebx

	cmpl	$ 0, %eax
	jg		3b // clean up loop

2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	.size	inner_kernel_gemm_sub_nt_2x2_lib4, .-inner_kernel_gemm_sub_nt_2x2_lib4
#endif





// common inner routine with file scope
//
// cholesky factorization 
//
// input arguments:
// eax  <- inv_diag_E
// ebx <- kn
// xmm0 <- [d00 d10]
// xmm1 <- [d01 d11]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_POTRF_2X2_VS_LIB4
#else
	.align 16
	.type inner_edge_potrf_2x2_vs_lib4, @function
inner_edge_potrf_2x2_vs_lib4:
#endif
	
	xorpd			%xmm7, %xmm7 // 0.0

	movsd			%xmm0, %xmm6
	ucomisd			%xmm7, %xmm6 // d_00 > 0.0 ?
	jbe				1f
	sqrtsd			%xmm6, %xmm6
	movsd			.LC00, %xmm5 // 1.0
	divsd			%xmm6, %xmm5
2:
	cmpl			$ 2, %ebx
	movsd			%xmm5, 0(%eax)
	movddup			%xmm5, %xmm5
	mulpd			%xmm5, %xmm0

	jl				0f // ret

	movapd			%xmm0, %xmm6
	shufpd			$ 0x3, %xmm6, %xmm6
	movapd			%xmm6, %xmm5
	mulpd			%xmm0, %xmm6
	subpd			%xmm6, %xmm1
	movapd			%xmm1, %xmm6
	shufpd			$ 0x3, %xmm6, %xmm6 // 0x1 ???
	ucomisd			%xmm7, %xmm6 // d_11 > 0.0 ?
	jbe				3f
	sqrtsd			%xmm6, %xmm6
	movsd			.LC00, %xmm5 // 1.0
	divsd			%xmm6, %xmm5
4:
	movsd			%xmm5, 8(%eax)
	movddup			%xmm5, %xmm5
	mulpd			%xmm5, %xmm1

	jmp		0f
	
1:
	xorpd	%xmm5, %xmm5
	jmp		2b

3:
	xorpd	%xmm5, %xmm5
	jmp		4b

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_edge_potrf_2x2_vs_lib4, .-inner_edge_potrf_2x2_vs_lib4
#endif





// common inner routine with file scope
//
// blend for generic alpha and beta
//
// input arguments:
// eax   <- alpha
// ebx   <- beta
// ecx   <- C
// xmm0 <- [d00 d10]
// xmm1 <- [d01 d11]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_2X2_LIB4
#else
	.align 16
	.type inner_scale_ab_2x2_lib4, @function
inner_scale_ab_2x2_lib4:
#endif

	// alpha
	movddup		0(%eax), %xmm7

	mulpd		%xmm7, %xmm0
	mulpd		%xmm7, %xmm1

	// beta
	movddup		0(%ebx), %xmm6

	xorpd		%xmm7, %xmm7 // 0.0

	ucomisd		%xmm7, %xmm6 // beta==0.0 ?
	je			0f // end

	movapd		0(%ecx), %xmm7
	mulpd		%xmm6, %xmm7
	addpd		%xmm7, %xmm0
	movapd		32(%ecx), %xmm7
	mulpd		%xmm6, %xmm7
	addpd		%xmm7, %xmm1

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_scale_ab_2x2_lib4, .-inner_scale_ab_2x2_lib4
#endif





// common inner routine with file scope
//
// blend for generic alpha=1 and beta=1
//
// input arguments:
// eax   <- C
// xmm0 <- [d00 d10]
// xmm1 <- [d01 d11]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_11_2X2_LIB4
#else
	.align 16
	.type inner_scale_11_2x2_lib4, @function
inner_scale_11_2x2_lib4:
#endif

	movapd		0(%eax), %xmm7
	addpd		%xmm7, %xmm0
	movapd		32(%eax), %xmm7
	addpd		%xmm7, %xmm1

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_scale_11_2x2_lib4, .-inner_scale_11_2x2_lib4
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// eax  <- D
// xmm0 <- [d00 d10]
// xmm1 <- [d01 d11]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_2X2_LIB4
#else
	.align 16
	.type inner_store_2x2_lib4, @function
inner_store_2x2_lib4:
#endif

	movapd	%xmm0,  0(%eax)
	movapd	%xmm1, 32(%eax)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_store_2x2_lib4, .-inner_store_2x2_lib4
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// eax  <- D
// ebx  <- m1
// ecx  <- n1
// xmm0 <- [d00 d10]
// xmm1 <- [d01 d11]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_2X2_VS_LIB4
#else
	.align 16
	.type inner_store_2x2_vs_lib4, @function
inner_store_2x2_vs_lib4:
#endif

	cmpl		$ 2, %ebx
	jge			0f

	// km==1
	movsd		%xmm0,  0(%eax)
	cmpl		$ 2, %ecx
	jl			4f // end
	movsd		%xmm1, 32(%eax)

	jmp		4f

0:
	// km==2
	movapd		%xmm0,  0(%eax)
	cmpl		$ 2, %ecx
	jl			4f // end
	movapd		%xmm1, 32(%eax)

4:



#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_store_2x2_vs_lib4, .-inner_store_2x2_vs_lib4
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// eax  <- D
// xmm0 <- [d00 d10]
// xmm1 <- [d01 d11]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_2X2_LIB4
#else
	.align 16
	.type inner_store_l_2x2_lib4, @function
inner_store_l_2x2_lib4:
#endif

	movapd	%xmm0,  0(%eax)
	movhpd	%xmm1, 40(%eax)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_store_l_2x2_lib4, .-inner_store_l_2x2_lib4
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// eax  <- D
// ebx  <- m1
// ecx  <- n1
// xmm0 <- [d00 d10]
// xmm1 <- [d01 d11]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_2X2_VS_LIB4
#else
	.align 16
	.type inner_store_l_2x2_vs_lib4, @function
inner_store_l_2x2_vs_lib4:
#endif

	cmpl		$ 2, %ebx
	jge			0f

	// km==1
	movsd		%xmm0,  0(%eax)

	jmp		4f

0:
	// km==2
	movapd		%xmm0,  0(%eax)
	cmpl		$ 2, %ecx
	jl			4f // end
	movhpd		%xmm1, 40(%eax)

4:



#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_store_l_2x2_vs_lib4, .-inner_store_l_2x2_vs_lib4
#endif





//                                 1      2              3          4          5             6          7
// void kernel_dsyrk_nt_l_2x2_lib4(int k, double *alpha, double *A, double *B, double *beta, double *C, double *D);

	.align 16
	.globl kernel_dsyrk_nt_l_2x2_lib4
	.type kernel_dsyrk_nt_l_2x2_lib4, @function
kernel_dsyrk_nt_l_2x2_lib4:

	PROLOGUE

	// zero accumulation registers

	xorpd	%xmm0, %xmm0
	movapd	%xmm0, %xmm1


	// call inner gemm kernel nt

	movl	ARG1, %eax // k
	movl	ARG3, %ebx  // A
	movl	ARG4, %ecx  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_2X2_LIB4
#else
	call inner_kernel_gemm_add_nt_2x2_lib4
#endif


	// call inner blend scale

	movl	ARG2, %eax // alpha
	movl	ARG5, %ebx // beta
	movl	ARG6, %ecx   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_2X2_LIB4
#else
	call inner_scale_ab_2x2_lib4
#endif


	// store n

	movl	ARG7, %eax // D

#if MACRO_LEVEL>=1
	INNER_STORE_L_2X2_LIB4
#else
	call inner_store_l_2x2_lib4
#endif

	EPILOGUE

	ret

	.size	kernel_dsyrk_nt_l_2x2_lib4, .-kernel_dsyrk_nt_l_2x2_lib4





//                                    1      2              3          4          5             6          7          8       9
// void kernel_dsyrk_nt_l_2x2_vs_lib4(int k, double *alpha, double *A, double *B, double *beta, double *C, double *D, int m1, int n1);

	.align 16
	.globl kernel_dsyrk_nt_l_2x2_vs_lib4
	.type kernel_dsyrk_nt_l_2x2_vs_lib4, @function
kernel_dsyrk_nt_l_2x2_vs_lib4:

	PROLOGUE

	// zero accumulation registers

	xorpd	%xmm0, %xmm0
	movapd	%xmm0, %xmm1


	// call inner gemm kernel nt

	movl	ARG1, %eax // k
	movl	ARG3, %ebx  // A
	movl	ARG4, %ecx  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_2X2_LIB4
#else
	call inner_kernel_gemm_add_nt_2x2_lib4
#endif


	// call inner blend scale

	movl	ARG2, %eax // alpha
	movl	ARG5, %ebx // beta
	movl	ARG6, %ecx   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_2X2_LIB4
#else
	call inner_scale_ab_2x2_lib4
#endif


	// store n

	movl	ARG7, %eax // D
	movl	ARG8, %ebx // m1
	movl	ARG9, %ecx // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_2X2_VS_LIB4
#else
	call inner_store_l_2x2_vs_lib4
#endif

	EPILOGUE

	ret

	.size	kernel_dsyrk_nt_l_2x2_vs_lib4, .-kernel_dsyrk_nt_l_2x2_vs_lib4





//                                  1      2          3          4          5          6
// void kernel_dpotrf_nt_l_2x2_lib4(int k, double *A, double *B, double *C, double *D, double *inv_diag_D);

	.align 16
	.globl kernel_dpotrf_nt_l_2x2_lib4
	.type kernel_dpotrf_nt_l_2x2_lib4, @function
kernel_dpotrf_nt_l_2x2_lib4:
	
	PROLOGUE

	// zero accumulation registers

	xorpd	%xmm0, %xmm0
	movapd	%xmm0, %xmm1


	// call inner dgemm kernel nt

	movl	ARG1, %eax
	movl	ARG2, %ebx
	movl	ARG3, %ecx

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_SUB_NT_2X2_LIB4
#else
	call inner_kernel_gemm_sub_nt_2x2_lib4
#endif


	// call inner blender_loader nn

	movl	ARG4, %eax // C

#if MACRO_LEVEL>=1
	INNER_SCALE_11_2X2_LIB4
#else
	call inner_scale_11_2x2_lib4
#endif


	// factorization

	movl	ARG6, %eax  // inv_diag_D 
	movl	$ 4, %ebx // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_POTRF_2X2_VS_LIB4
#else
	call inner_edge_potrf_2x2_vs_lib4
#endif


	// store

	movl	ARG5, %eax // D

#if MACRO_LEVEL>=1
	INNER_STORE_L_2X2_LIB4
#else
	call inner_store_l_2x2_lib4
#endif


	EPILOGUE

	ret

	.size	kernel_dpotrf_nt_l_2x2_lib4, .-kernel_dpotrf_nt_l_2x2_lib4





//                                     1      2          3          4          5          6                   7       8
// void kernel_dpotrf_nt_l_2x2_vs_lib4(int k, double *A, double *B, double *C, double *D, double *inv_diag_D, int m1, int n1);

	.align 16
	.globl kernel_dpotrf_nt_l_2x2_vs_lib4
	.type kernel_dpotrf_nt_l_2x2_vs_lib4, @function
kernel_dpotrf_nt_l_2x2_vs_lib4:
	
	PROLOGUE

	// zero accumulation registers

	xorpd	%xmm0, %xmm0
	movapd	%xmm0, %xmm1


	// call inner dgemm kernel nt

	movl	ARG1, %eax
	movl	ARG2, %ebx
	movl	ARG3, %ecx

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_SUB_NT_2X2_LIB4
#else
	call inner_kernel_gemm_sub_nt_2x2_lib4
#endif


	// call inner blender_loader nn

	movl	ARG4, %eax // C

#if MACRO_LEVEL>=1
	INNER_SCALE_11_2X2_LIB4
#else
	call inner_scale_11_2x2_lib4
#endif


	// factorization

	movl	ARG6, %eax  // inv_diag_D 
	movl	ARG8, %ebx // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_POTRF_2X2_VS_LIB4
#else
	call inner_edge_potrf_2x2_vs_lib4
#endif


	// store

	movl	ARG5, %eax // D
	movl	ARG7, %ebx // m1
	movl	ARG8, %ecx // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_2X2_VS_LIB4
#else
	call inner_store_l_2x2_vs_lib4
#endif


	EPILOGUE

	ret

	.size	kernel_dpotrf_nt_l_2x2_vs_lib4, .-kernel_dpotrf_nt_l_2x2_vs_lib4





//                                        1       2           3           4       5           6           7          8          9
// void kernel_dsyrk_dpotrf_nt_l_2x2_lib4(int kp, double *Ap, double *Bp, int km, double *Am, double *Bm, double *C, double *D, double *inv_diag_D);

	.align 16
	.globl kernel_dsyrk_dpotrf_nt_l_2x2_lib4
	.type kernel_dsyrk_dpotrf_nt_l_2x2_lib4, @function
kernel_dsyrk_dpotrf_nt_l_2x2_lib4:
	
	PROLOGUE

	// zero accumulation registers

	xorpd	%xmm0, %xmm0
	movapd	%xmm0, %xmm1


	// call inner dgemm kernel nt add

	movl	ARG1, %eax // kp
	movl	ARG2, %ebx  // Ap
	movl	ARG3, %ecx  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_2X2_LIB4
#else
	call inner_kernel_gemm_add_nt_2x2_lib4
#endif


	// call inner dgemm kernel nt sub

	movl	ARG4, %eax // km
	movl	ARG5, %ebx   // Am
	movl	ARG6, %ecx   // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_SUB_NT_2X2_LIB4
#else
	call inner_kernel_gemm_sub_nt_2x2_lib4
#endif


	// call inner blender_loader nn

	movl	ARG7, %eax   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_11_2X2_LIB4
#else
	call inner_scale_11_2x2_lib4
#endif


	// factorization

	movl	ARG9, %eax  // inv_diag_D 
	movl	$ 4, %ebx

#if MACRO_LEVEL>=1
	INNER_EDGE_POTRF_2X2_VS_LIB4
#else
	call inner_edge_potrf_2x2_vs_lib4
#endif


	// store

	movl	ARG8, %eax  // D 

#if MACRO_LEVEL>=1
	INNER_STORE_L_2X2_LIB4
#else
	call inner_store_l_2x2_lib4
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dsyrk_dpotrf_nt_l_2x2_lib4, .-kernel_dsyrk_dpotrf_nt_l_2x2_lib4
#endif





//                                           1       2           3           4       5           6           7          8          9                   10      11
// void kernel_dsyrk_dpotrf_nt_l_2x2_vs_lib4(int kp, double *Ap, double *Bp, int km, double *Am, double *Bm, double *C, double *D, double *inv_diag_D, int km, int kn);

	.align 16
	.globl kernel_dsyrk_dpotrf_nt_l_2x2_vs_lib4
	.type kernel_dsyrk_dpotrf_nt_l_2x2_vs_lib4, @function
kernel_dsyrk_dpotrf_nt_l_2x2_vs_lib4:
	
	PROLOGUE

	// zero accumulation registers

	xorpd	%xmm0, %xmm0
	movapd	%xmm0, %xmm1


	// call inner dgemm kernel nt add

	movl	ARG1, %eax // kp
	movl	ARG2, %ebx  // Ap
	movl	ARG3, %ecx  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_2X2_LIB4
#else
	call inner_kernel_gemm_add_nt_2x2_lib4
#endif


	// call inner dgemm kernel nt sub

	movl	ARG4, %eax // km
	movl	ARG5, %ebx   // Am
	movl	ARG6, %ecx   // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_SUB_NT_2X2_LIB4
#else
	call inner_kernel_gemm_sub_nt_2x2_lib4
#endif


	// call inner blender_loader nn

	movl	ARG7, %eax   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_11_2X2_LIB4
#else
	call inner_scale_11_2x2_lib4
#endif


	// factorization

	movl	ARG9, %eax  // inv_diag_D 
	movl	ARG11, %ebx // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_POTRF_2X2_VS_LIB4
#else
	call inner_edge_potrf_2x2_vs_lib4
#endif


	// store

	movl	ARG8, %eax  // D 
	movl	ARG10, %ebx // km 
	movl	ARG11, %ecx // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_L_2X2_VS_LIB4
#else
	call inner_store_l_2x2_vs_lib4
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dsyrk_dpotrf_nt_l_2x2_vs_lib4, .-kernel_dsyrk_dpotrf_nt_l_2x2_vs_lib4
#endif





	// read-only data
	.section	.rodata.cst32,"aM",@progbits,32

	.align 32
.LC00: // { 1.0 1.0 1.0 1.0 }
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248




