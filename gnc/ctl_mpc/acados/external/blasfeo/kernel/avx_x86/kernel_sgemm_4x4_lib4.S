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
// xmm0  <- [d00 d11 d22 d33]
// xmm1  <- [d01 d10 d23 d32]
// xmm2  <- [d03 d12 d21 d30]
// xmm3  <- [d02 d13 d20 d31]

//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NT_4X4_LIB4
#else
	.align 16
	.type inner_kernel_gemm_add_nt_4x4_lib4, @function
inner_kernel_gemm_add_nt_4x4_lib4:
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
	vmovaps 		0(%ecx), %xmm5 // B
	vmovaps 		0(%ebx), %xmm4 // A
	vshufps			$ 0xb1, %xmm5, %xmm5, %xmm6
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm0, %xmm7, %xmm0
	vshufps			$ 0x1b, %xmm5, %xmm5, %xmm7
	vshufps			$ 0x4e, %xmm5, %xmm5, %xmm5
	vmulps			%xmm4, %xmm6, %xmm6
	vaddps			%xmm1, %xmm6, %xmm1
	vmulps			%xmm4, %xmm7, %xmm7
	vaddps			%xmm2, %xmm7, %xmm2
	vmulps			%xmm4, %xmm5, %xmm5
	vaddps			%xmm3, %xmm5, %xmm3

	// unroll 1
	vmovaps 		16(%ecx), %xmm5 // B
	vmovaps 		16(%ebx), %xmm4 // A
	vshufps			$ 0xb1, %xmm5, %xmm5, %xmm6
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm0, %xmm7, %xmm0
	vshufps			$ 0x1b, %xmm5, %xmm5, %xmm7
	vshufps			$ 0x4e, %xmm5, %xmm5, %xmm5
	vmulps			%xmm4, %xmm6, %xmm6
	vaddps			%xmm1, %xmm6, %xmm1
	vmulps			%xmm4, %xmm7, %xmm7
	vaddps			%xmm2, %xmm7, %xmm2
	vmulps			%xmm4, %xmm5, %xmm5
	vaddps			%xmm3, %xmm5, %xmm3

	// unroll 2
	vmovaps 		32(%ecx), %xmm5 // B
	vmovaps 		32(%ebx), %xmm4 // A
	vshufps			$ 0xb1, %xmm5, %xmm5, %xmm6
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm0, %xmm7, %xmm0
	vshufps			$ 0x1b, %xmm5, %xmm5, %xmm7
	vshufps			$ 0x4e, %xmm5, %xmm5, %xmm5
	vmulps			%xmm4, %xmm6, %xmm6
	vaddps			%xmm1, %xmm6, %xmm1
	vmulps			%xmm4, %xmm7, %xmm7
	vaddps			%xmm2, %xmm7, %xmm2
	vmulps			%xmm4, %xmm5, %xmm5
	vaddps			%xmm3, %xmm5, %xmm3

	// unroll 3
	vmovaps 		48(%ecx), %xmm5 // B
	vmovaps 		48(%ebx), %xmm4 // A
	vshufps			$ 0xb1, %xmm5, %xmm5, %xmm6
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm0, %xmm7, %xmm0
	vshufps			$ 0x1b, %xmm5, %xmm5, %xmm7
	vshufps			$ 0x4e, %xmm5, %xmm5, %xmm5
	vmulps			%xmm4, %xmm6, %xmm6
	vaddps			%xmm1, %xmm6, %xmm1
	vmulps			%xmm4, %xmm7, %xmm7
	vaddps			%xmm2, %xmm7, %xmm2
	vmulps			%xmm4, %xmm5, %xmm5
	vaddps			%xmm3, %xmm5, %xmm3

	subl	$ 4, %eax
	addl	$ 64, %ecx
	addl	$ 64, %ebx

	cmpl	$ 4, %eax
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %eax
	jle		4f // clean1

	// unroll 0
	vmovaps 		0(%ecx), %xmm5 // B
	vmovaps 		0(%ebx), %xmm4 // A
	vshufps			$ 0xb1, %xmm5, %xmm5, %xmm6
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm0, %xmm7, %xmm0
	vshufps			$ 0x1b, %xmm5, %xmm5, %xmm7
	vshufps			$ 0x4e, %xmm5, %xmm5, %xmm5
	vmulps			%xmm4, %xmm6, %xmm6
	vaddps			%xmm1, %xmm6, %xmm1
	vmulps			%xmm4, %xmm7, %xmm7
	vaddps			%xmm2, %xmm7, %xmm2
	vmulps			%xmm4, %xmm5, %xmm5
	vaddps			%xmm3, %xmm5, %xmm3

	// unroll 1
	vmovaps 		16(%ecx), %xmm5 // B
	vmovaps 		16(%ebx), %xmm4 // A
	vshufps			$ 0xb1, %xmm5, %xmm5, %xmm6
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm0, %xmm7, %xmm0
	vshufps			$ 0x1b, %xmm5, %xmm5, %xmm7
	vshufps			$ 0x4e, %xmm5, %xmm5, %xmm5
	vmulps			%xmm4, %xmm6, %xmm6
	vaddps			%xmm1, %xmm6, %xmm1
	vmulps			%xmm4, %xmm7, %xmm7
	vaddps			%xmm2, %xmm7, %xmm2
	vmulps			%xmm4, %xmm5, %xmm5
	vaddps			%xmm3, %xmm5, %xmm3

	// unroll 2
	vmovaps 		32(%ecx), %xmm5 // B
	vmovaps 		32(%ebx), %xmm4 // A
	vshufps			$ 0xb1, %xmm5, %xmm5, %xmm6
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm0, %xmm7, %xmm0
	vshufps			$ 0x1b, %xmm5, %xmm5, %xmm7
	vshufps			$ 0x4e, %xmm5, %xmm5, %xmm5
	vmulps			%xmm4, %xmm6, %xmm6
	vaddps			%xmm1, %xmm6, %xmm1
	vmulps			%xmm4, %xmm7, %xmm7
	vaddps			%xmm2, %xmm7, %xmm2
	vmulps			%xmm4, %xmm5, %xmm5
	vaddps			%xmm3, %xmm5, %xmm3

	// unroll 3
	vmovaps 		48(%ecx), %xmm5 // B
	vmovaps 		48(%ebx), %xmm4 // A
	vshufps			$ 0xb1, %xmm5, %xmm5, %xmm6
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm0, %xmm7, %xmm0
	vshufps			$ 0x1b, %xmm5, %xmm5, %xmm7
	vshufps			$ 0x4e, %xmm5, %xmm5, %xmm5
	vmulps			%xmm4, %xmm6, %xmm6
	vaddps			%xmm1, %xmm6, %xmm1
	vmulps			%xmm4, %xmm7, %xmm7
	vaddps			%xmm2, %xmm7, %xmm2
	vmulps			%xmm4, %xmm5, %xmm5
	vaddps			%xmm3, %xmm5, %xmm3

	subl	$ 4, %eax
	addl	$ 64, %ecx
	addl	$ 64, %ebx

//	cmpl	$ 3, %eax
	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %eax
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovaps 		0(%ecx), %xmm5 // B
	vmovaps 		0(%ebx), %xmm4 // A
	vshufps			$ 0xb1, %xmm5, %xmm5, %xmm6
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm0, %xmm7, %xmm0
	vshufps			$ 0x1b, %xmm5, %xmm5, %xmm7
	vshufps			$ 0x4e, %xmm5, %xmm5, %xmm5
	vmulps			%xmm4, %xmm6, %xmm6
	vaddps			%xmm1, %xmm6, %xmm1
	vmulps			%xmm4, %xmm7, %xmm7
	vaddps			%xmm2, %xmm7, %xmm2
	vmulps			%xmm4, %xmm5, %xmm5
	vaddps			%xmm3, %xmm5, %xmm3

	subl	$ 1, %eax
	addl	$ 16, %ecx
	addl	$ 16, %ebx

	cmpl	$ 0, %eax
	jg		3b // clean up loop

2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	.size	inner_kernel_gemm_add_nt_4x4_lib4, .-inner_kernel_gemm_add_nt_4x4_lib4
#endif





// common inner routine with file scope
//
// input arguments:
// eax  <- k
// ebx   <- A
// ecx   <- B
// edx   <- 4*sdb*sizeof(float)
// xmm0  <- [d00 d11 d22 d33]
// xmm1  <- [d01 d10 d23 d32]
// xmm2  <- [d03 d12 d21 d30]
// xmm3  <- [d02 d13 d20 d31]

//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_4X4_LIB4
#else
	.p2align 4,,15
	.type inner_kernel_gemm_add_nn_4x4_lib4, @function
inner_kernel_gemm_add_nn_4x4_lib4:
#endif

	cmpl	$ 0, %eax
	jle		2f // return

	// preload

	cmpl	$ 4, %eax
	jle		0f // consider clean-up loop

	// main loop
	.align 8
1: // main loop

	prefetcht0	0(%ecx, %edx, 1) // software prefetch

	// unroll 0
	vmovaps 		0(%ebx), %xmm4 // A
	vbroadcastss	0(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm0, %xmm7, %xmm0
	vbroadcastss	16(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm1, %xmm7, %xmm1
	vbroadcastss	32(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm2, %xmm7, %xmm2
	vbroadcastss	48(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm3, %xmm7, %xmm3

	// unroll 1
	vmovaps 		16(%ebx), %xmm4 // A
	vbroadcastss	4(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm0, %xmm7, %xmm0
	vbroadcastss	20(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm1, %xmm7, %xmm1
	vbroadcastss	36(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm2, %xmm7, %xmm2
	vbroadcastss	52(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm3, %xmm7, %xmm3

	// unroll 2
	vmovaps 		32(%ebx), %xmm4 // A
	vbroadcastss	8(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm0, %xmm7, %xmm0
	vbroadcastss	24(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm1, %xmm7, %xmm1
	vbroadcastss	40(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm2, %xmm7, %xmm2
	vbroadcastss	56(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm3, %xmm7, %xmm3

	// unroll 3
	vmovaps 		48(%ebx), %xmm4 // A
	vbroadcastss	12(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm0, %xmm7, %xmm0
	vbroadcastss	28(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm1, %xmm7, %xmm1
	vbroadcastss	44(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm2, %xmm7, %xmm2
	vbroadcastss	60(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm3, %xmm7, %xmm3

	subl	$ 4, %eax
	addl	$ 64, %ebx
	addl	%edx, %ecx

	cmpl	$ 4, %eax
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %eax
	jle		4f // clean1

	// unroll 0
	vmovaps 		0(%ebx), %xmm4 // A
	vbroadcastss	0(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm0, %xmm7, %xmm0
	vbroadcastss	16(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm1, %xmm7, %xmm1
	vbroadcastss	32(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm2, %xmm7, %xmm2
	vbroadcastss	48(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm3, %xmm7, %xmm3

	// unroll 1
	vmovaps 		16(%ebx), %xmm4 // A
	vbroadcastss	4(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm0, %xmm7, %xmm0
	vbroadcastss	20(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm1, %xmm7, %xmm1
	vbroadcastss	36(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm2, %xmm7, %xmm2
	vbroadcastss	52(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm3, %xmm7, %xmm3

	// unroll 2
	vmovaps 		32(%ebx), %xmm4 // A
	vbroadcastss	8(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm0, %xmm7, %xmm0
	vbroadcastss	24(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm1, %xmm7, %xmm1
	vbroadcastss	40(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm2, %xmm7, %xmm2
	vbroadcastss	56(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm3, %xmm7, %xmm3

	// unroll 3
	vmovaps 		48(%ebx), %xmm4 // A
	vbroadcastss	12(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm0, %xmm7, %xmm0
	vbroadcastss	28(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm1, %xmm7, %xmm1
	vbroadcastss	44(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm2, %xmm7, %xmm2
	vbroadcastss	60(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm3, %xmm7, %xmm3

	subl	$ 4, %eax
	addl	$ 64, %ebx
	addl	%edx, %ecx

//	cmpl	$ 3, %eax
	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %eax
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovaps 		0(%ebx), %xmm4 // A
	vbroadcastss	0(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm0, %xmm7, %xmm0
	vbroadcastss	16(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm1, %xmm7, %xmm1
	vbroadcastss	32(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm2, %xmm7, %xmm2
	vbroadcastss	48(%ecx), %xmm5 // B
	vmulps			%xmm4, %xmm5, %xmm7
	vaddps			%xmm3, %xmm7, %xmm3

	subl	$ 1, %eax
	addl	$ 16, %ebx
	addl	$ 4, %ecx

	cmpl	$ 0, %eax
	jg		3b // clean up loop

2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	.size	inner_kernel_gemm_add_nn_4x4_lib4, .-inner_kernel_gemm_add_nn_4x4_lib4
#endif





// common inner routine with file scope
//
// edge for B unaligned, A aligned
//
// input arguments:
// eax   <- k
// ebx   <- A
// ecx   <- B
// edx   <- bs*sdb*sizeof(float)
// esi   <- offB
// xmm0  <- [d00 d11 d22 d33]
// xmm1  <- [d01 d10 d23 d32]
// xmm2  <- [d03 d12 d21 d30]
// xmm3  <- [d02 d13 d20 d31]

//
// output arguments:


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_GEMM_ADD_NN_4X4_LIB4
#else
	.p2align 4,,15
	.type inner_edge_gemm_add_nn_4x4_lib4, @function
inner_edge_gemm_add_nn_4x4_lib4:
#endif

	cmpl			$ 0, %esi				// offset==0
	jle				2f						// end

	cmpl			$ 0, %eax				// k==0
	jle				2f						// end

	movl			%esi, %edi				// load offsetB
	sall			$ 3, %edi				// offsetB*sizeof(float)
	addl			%edi, %ecx				// B+offsetB*sizeof(float)

	movl			$ 4, %edi				// load 4
	subl			%esi, %edi				// 4-offsetB
	cmpl			%eax, %edi				// k > 4-offsetB
	cmovgl			%eax, %edi				// kend=min(k,4-offsetB)

1:
	vmovaps			0(%ebx), %xmm5			// load A

	vbroadcastss	0(%ecx), %xmm4			// B_off
	vmulps			%xmm5, %xmm4, %xmm7		// mul
	vaddps			%xmm7, %xmm0, %xmm0		// store

	vbroadcastss	16(%ecx), %xmm4			// B_off+4
	vmulps			%xmm5, %xmm4, %xmm7		// mul
	vaddps			%xmm7, %xmm1, %xmm1		// store

	vbroadcastss	32(%ecx), %xmm4			// B_off+8
	vmulps			%xmm5, %xmm4, %xmm7		// mul
	vaddps			%xmm7, %xmm2, %xmm2		// store

	vbroadcastss	48(%ecx), %xmm4			// B_off+12
	vmulps			%xmm5, %xmm4, %xmm7		// mul
	vaddps			%xmm7, %xmm3, %xmm3		// store

	subl			$ 1, %eax				// k=-1
	subl			$ 1, %edi				// k_panel=-1
	addl			$ 16, %ebx				// A=+bs
	addl			$ 4, %ecx				// B=+1

	cmpl			$ 0, %edi				// if k_panel=0
	jg				1b						// loop 1

	cmpl			$ 0, %eax				// if k=0
	jle				2f						// end

	addl			%edx, %ecx				// B=Boff+sdb*bs
	subl			$ 16, %ecx				// B-=4*sizeof(float) (loop+offsetB)

2:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_edge_gemm_add_nn_4x4_lib4, .-inner_edge_gemm_add_nn_4x4_lib4
#endif





/// common inner routine with file scope
//
// blend for generic alpha and beta
//
// input arguments:
// eax   <- alpha
// ebx   <- beta
// ecx   <- C
// xmm0 <- [d00 d11 d22 d33]
// xmm1 <- [d01 d10 d23 d32]
// xmm2 <- [d03 d12 d21 d30]
// xmm3 <- [d02 d13 d20 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_4X4_LIB4
#else
	.align 16
	.type inner_scale_ab_4x4_lib4, @function
inner_scale_ab_4x4_lib4:
#endif

	// alpha
	vbroadcastss	0(%eax), %xmm7

	vmulps		%xmm0, %xmm7, %xmm0
	vmulps		%xmm1, %xmm7, %xmm1
	vmulps		%xmm2, %xmm7, %xmm2
	vmulps		%xmm3, %xmm7, %xmm3

	// beta
	vbroadcastss	0(%ebx), %xmm6

	vxorps		%xmm7, %xmm7, %xmm7 // 0.0

	vucomiss	%xmm7, %xmm6 // beta==0.0 ?
	je			0f // end

	vmovaps		0(%ecx), %xmm7
	vmulps		%xmm7, %xmm6, %xmm7
	vaddps		%xmm0, %xmm7, %xmm0
	vmovaps		16(%ecx), %xmm7
	vmulps		%xmm7, %xmm6, %xmm7
	vaddps		%xmm1, %xmm7, %xmm1
	vmovaps		32(%ecx), %xmm7
	vmulps		%xmm7, %xmm6, %xmm7
	vaddps		%xmm2, %xmm7, %xmm2
	vmovaps		48(%ecx), %xmm7
	vmulps		%xmm7, %xmm6, %xmm7
	vaddps		%xmm3, %xmm7, %xmm3

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_scale_ab_4x4_lib4, .-inner_scale_ab_4x4_lib4
#endif





// common inner routine with file scope
//
// blend for generic alpha and beta
//
// input arguments:
// eax   <- alpha
// ebx   <- beta
// ecx   <- C
// xmm0 <- [d00 d11 d22 d33]
// xmm1 <- [d01 d10 d23 d32]
// xmm2 <- [d03 d12 d21 d30]
// xmm3 <- [d02 d13 d20 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_SCALE_AB_4X4_LIB4
#else
	.align 16
	.type inner_blend_scale_ab_4x4_lib4, @function
inner_blend_scale_ab_4x4_lib4:
#endif

	vblendps	$ 0x5, %xmm0, %xmm1, %xmm4
	vblendps	$ 0xa, %xmm0, %xmm1, %xmm5
	vblendps	$ 0x5, %xmm2, %xmm3, %xmm6
	vblendps	$ 0xa, %xmm2, %xmm3, %xmm7

	vblendps	$ 0xc, %xmm7, %xmm4, %xmm0
	vblendps	$ 0xc, %xmm4, %xmm7, %xmm2
	vblendps	$ 0xc, %xmm6, %xmm5, %xmm1
	vblendps	$ 0xc, %xmm5, %xmm6, %xmm3

	// alpha
	vbroadcastss	0(%eax), %xmm7

	vmulps		%xmm0, %xmm7, %xmm0
	vmulps		%xmm1, %xmm7, %xmm1
	vmulps		%xmm2, %xmm7, %xmm2
	vmulps		%xmm3, %xmm7, %xmm3

	// beta
	vbroadcastss	0(%ebx), %xmm6

	vxorps		%xmm7, %xmm7, %xmm7 // 0.0

	vucomiss	%xmm7, %xmm6 // beta==0.0 ?
	je			0f // end

	vmovaps		0(%ecx), %xmm7
	vmulps		%xmm7, %xmm6, %xmm7
	vaddps		%xmm0, %xmm7, %xmm0
	vmovaps		16(%ecx), %xmm7
	vmulps		%xmm7, %xmm6, %xmm7
	vaddps		%xmm1, %xmm7, %xmm1
	vmovaps		32(%ecx), %xmm7
	vmulps		%xmm7, %xmm6, %xmm7
	vaddps		%xmm2, %xmm7, %xmm2
	vmovaps		48(%ecx), %xmm7
	vmulps		%xmm7, %xmm6, %xmm7
	vaddps		%xmm3, %xmm7, %xmm3

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_blend_scale_ab_4x4_lib4, .-inner_blend_scale_ab_4x4_lib4
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// eax  <- D
// xmm0 <- [d00 d11 d22 d33]
// xmm1 <- [d01 d10 d23 d32]
// xmm2 <- [d03 d12 d21 d30]
// xmm3 <- [d02 d13 d20 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X4_LIB4
#else
	.align 16
	.type inner_store_4x4_lib4, @function
inner_store_4x4_lib4:
#endif

	vmovaps	%xmm0,  0(%eax)
	vmovaps %xmm1, 16(%eax)
	vmovaps %xmm2, 32(%eax)
	vmovaps %xmm3, 48(%eax)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_store_4x4_lib4, .-inner_store_4x4_lib4
#endif





// common inner routine with file scope
//
// store n vs
//
// input arguments:
// eax  <- D
// ebx  <- km
// ecx  <- kn
// xmm0 <- [d00 d11 d22 d33]
// xmm1 <- [d01 d10 d23 d32]
// xmm2 <- [d03 d12 d21 d30]
// xmm3 <- [d02 d13 d20 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X4_VS_LIB4
#else
	.align 16
	.type inner_store_4x4_vs_lib4, @function
inner_store_4x4_vs_lib4:
#endif
	
	// compute mask for rows
	vcvtsi2ss	%ebx, %xmm7, %xmm7
	vmovups		.LC00, %xmm6
	vshufps		$ 0x00, %xmm7, %xmm7, %xmm7
	vsubps		%xmm7, %xmm6, %xmm7

	// offset==0
	vmaskmovps	%xmm0, %xmm7,  0(%eax)
	cmpl		$ 2, %ecx
	jl			0f // end
	vmaskmovps	%xmm1, %xmm7, 16(%eax)
	cmpl		$ 3, %ecx
	jl			0f // end
	vmaskmovps	%xmm2, %xmm7, 32(%eax)
	je			0f // end
	vmaskmovps	%xmm3, %xmm7, 48(%eax)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_store_4x4_vs_lib4, .-inner_store_4x4_vs_lib4
#endif
#endif





//                               1      2              3          4          5             6          7
// void kernel_sgemm_nt_4x4_lib4(int k, float *alpha, float *A, float *B, float *beta, float *C, float *D);

	.align 16
	.globl kernel_sgemm_nt_4x4_lib4
	.type kernel_sgemm_nt_4x4_lib4, @function
kernel_sgemm_nt_4x4_lib4:

	PROLOGUE

	// zero accumulation registers

	vxorps	%xmm0, %xmm0, %xmm0
	vmovaps	%xmm0, %xmm1
	vmovaps	%xmm0, %xmm2
	vmovaps	%xmm0, %xmm3


	// call inner gemm kernel nt

	movl	ARG1, %eax // k
	movl	ARG3, %ebx  // A
	movl	ARG4, %ecx  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X4_LIB4
#else
	call inner_kernel_gemm_add_nt_4x4_lib4
#endif


	// call inner blend scale

	movl	ARG2, %eax // alpha
	movl	ARG5, %ebx // beta
	movl	ARG6, %ecx   // C

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_AB_4X4_LIB4
#else
	call inner_blend_scale_ab_4x4_lib4
#endif


	// store n

	movl	ARG7, %eax // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
	call inner_store_4x4_lib4
#endif

	EPILOGUE

	ret

	.size	kernel_sgemm_nt_4x4_lib4, .-kernel_sgemm_nt_4x4_lib4





//                                  1      2              3          4          5             6          7          8       9
// void kernel_sgemm_nt_4x4_vs_lib4(int k, float *alpha, float *A, float *B, float *beta, float *C, float *D, int m1, int n1);

	.align 16
	.globl kernel_sgemm_nt_4x4_vs_lib4
	.type kernel_sgemm_nt_4x4_vs_lib4, @function
kernel_sgemm_nt_4x4_vs_lib4:

	PROLOGUE

	// zero accumulation registers

	vxorps	%xmm0, %xmm0, %xmm0
	vmovaps	%xmm0, %xmm1
	vmovaps	%xmm0, %xmm2
	vmovaps	%xmm0, %xmm3


	// call inner gemm kernel nt

	movl	ARG1, %eax // k
	movl	ARG3, %ebx  // A
	movl	ARG4, %ecx  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X4_LIB4
#else
	call inner_kernel_gemm_add_nt_4x4_lib4
#endif


	// call inner blend scale

	movl	ARG2, %eax // alpha
	movl	ARG5, %ebx // beta
	movl	ARG6, %ecx   // C

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_AB_4X4_LIB4
#else
	call inner_blend_scale_ab_4x4_lib4
#endif


	// store n

	movl	ARG7, %eax // D
	movl	ARG8, %ebx // m1
	movl	ARG9, %ecx // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	call inner_store_4x4_vs_lib4
#endif

	EPILOGUE

	ret

	.size	kernel_sgemm_nt_4x4_vs_lib4, .-kernel_sgemm_nt_4x4_vs_lib4





//                               1      2              3          4            5          6        7             8
// void kernel_sgemm_nn_4x4_lib4(int k, double *alpha, double *A, double *B, int sdb, double *beta, double *C, double *D);

	.align 16
	.globl kernel_sgemm_nn_4x4_lib4
	.type kernel_sgemm_nn_4x4_lib4, @function
kernel_sgemm_nn_4x4_lib4:

	PROLOGUE

	// zero accumulation registers

	vxorps	%xmm0, %xmm0, %xmm0
	vmovaps	%xmm0, %xmm1
	vmovaps	%xmm0, %xmm2
	vmovaps	%xmm0, %xmm3


	// call inner gemm kernel nn

	movl	ARG1, %eax // k
	movl	ARG3, %ebx  // A
	movl	ARG4, %ecx  // B
	movl	ARG5, %edx // sdb
	sall	$ 4, %edx // 4*sdb*sizeof(float)

// TODO add offset to interface
//#if MACRO_LEVEL>=1
//	INNER_EDGE_GEMM_ADD_NN_4X4_LIB4
//#else
//	call inner_edge_gemm_add_nn_4x4_lib4
//#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X4_LIB4
#else
	call inner_kernel_gemm_add_nn_4x4_lib4
#endif


	// call inner blend

	movl	ARG2, %eax // alpha
	movl	ARG6, %ebx // beta
	movl	ARG7, %ecx   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	call inner_scale_ab_4x4_lib4
#endif


	// store n

	movl	ARG8, %eax // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
	call inner_store_4x4_lib4
#endif


	EPILOGUE

	ret

	.size	kernel_sgemm_nn_4x4_lib4, .-kernel_sgemm_nn_4x4_lib4





//                               1      2              3          4            5          6        7             8           9       10
// void kernel_sgemm_nn_4x4_vs_lib4(int k, double *alpha, double *A, double *B, int sdb, double *beta, double *C, double *D, int m1, int n1);

	.align 16
	.globl kernel_sgemm_nn_4x4_vs_lib4
	.type kernel_sgemm_nn_4x4_vs_lib4, @function
kernel_sgemm_nn_4x4_vs_lib4:

	PROLOGUE

	// zero accumulation registers

	vxorps	%xmm0, %xmm0, %xmm0
	vmovaps	%xmm0, %xmm1
	vmovaps	%xmm0, %xmm2
	vmovaps	%xmm0, %xmm3


	// call inner gemm kernel nn

	movl	ARG1, %eax // k
	movl	ARG3, %ebx  // A
	movl	ARG4, %ecx  // B
	movl	ARG5, %edx // sdb
	sall	$ 4, %edx // 4*sdb*sizeof(float)

// TODO add offset to interface
//#if MACRO_LEVEL>=1
//	INNER_EDGE_GEMM_ADD_NN_4X4_LIB4
//#else
//	call inner_edge_gemm_add_nn_4x4_lib4
//#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X4_LIB4
#else
	call inner_kernel_gemm_add_nn_4x4_lib4
#endif


	// call inner blend

	movl	ARG2, %eax // alpha
	movl	ARG6, %ebx // beta
	movl	ARG7, %ecx   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	call inner_scale_ab_4x4_lib4
#endif


	// store n

	movl	ARG8, %eax // D
	movl	ARG9, %ebx // m1
	movl	ARG10, %ecx // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	call inner_store_4x4_vs_lib4
#endif


	EPILOGUE

	ret

	.size	kernel_sgemm_nn_4x4_vs_lib4, .-kernel_sgemm_nn_4x4_vs_lib4





	// read-only data
	.section	.rodata.cst32,"aM",@progbits,32
	.align 16
.LC00: // { 3.5 2.5 1.5 0.5 }
	.long	1056964608
	.long	1069547520
	.long	1075838976
	.long	1080033280

	.section	.note.GNU-stack,"",@progbits

