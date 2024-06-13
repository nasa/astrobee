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
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d03 d12 d21 d30]
// ymm3  <- [d02 d13 d20 d31]

//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	.align 16
	.type inner_kernel_gemm_nt_4x4_lib4, @function
inner_kernel_gemm_nt_4x4_lib4:
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
	vmovapd 		0(%ebx), %ymm4 // A
	vbroadcastsd	0(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm0, %ymm7, %ymm0
	vbroadcastsd	8(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm1, %ymm7, %ymm1
	vbroadcastsd	16(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm2, %ymm7, %ymm2
	vbroadcastsd	24(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm3, %ymm7, %ymm3

	// unroll 1
	vmovapd 		32(%ebx), %ymm4 // A
	vbroadcastsd	32(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm0, %ymm7, %ymm0
	vbroadcastsd	40(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm1, %ymm7, %ymm1
	vbroadcastsd	48(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm2, %ymm7, %ymm2
	vbroadcastsd	56(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm3, %ymm7, %ymm3

	// unroll 2
	vmovapd 		64(%ebx), %ymm4 // A
	vbroadcastsd	64(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm0, %ymm7, %ymm0
	vbroadcastsd	72(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm1, %ymm7, %ymm1
	vbroadcastsd	80(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm2, %ymm7, %ymm2
	vbroadcastsd	88(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm3, %ymm7, %ymm3

	// unroll 3
	vmovapd 		96(%ebx), %ymm4 // A
	vbroadcastsd	96(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm0, %ymm7, %ymm0
	vbroadcastsd	104(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm1, %ymm7, %ymm1
	vbroadcastsd	112(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm2, %ymm7, %ymm2
	vbroadcastsd	120(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm3, %ymm7, %ymm3

	subl	$ 4, %eax
	addl	$ 128, %ecx
	addl	$ 128, %ebx

	cmpl	$ 4, %eax
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %eax
	jle		4f // clean1

	// unroll 0
	vmovapd 		0(%ebx), %ymm4 // A
	vbroadcastsd	0(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm0, %ymm7, %ymm0
	vbroadcastsd	8(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm1, %ymm7, %ymm1
	vbroadcastsd	16(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm2, %ymm7, %ymm2
	vbroadcastsd	24(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm3, %ymm7, %ymm3

	// unroll 1
	vmovapd 		32(%ebx), %ymm4 // A
	vbroadcastsd	32(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm0, %ymm7, %ymm0
	vbroadcastsd	40(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm1, %ymm7, %ymm1
	vbroadcastsd	48(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm2, %ymm7, %ymm2
	vbroadcastsd	56(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm3, %ymm7, %ymm3

	// unroll 2
	vmovapd 		64(%ebx), %ymm4 // A
	vbroadcastsd	64(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm0, %ymm7, %ymm0
	vbroadcastsd	72(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm1, %ymm7, %ymm1
	vbroadcastsd	80(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm2, %ymm7, %ymm2
	vbroadcastsd	88(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm3, %ymm7, %ymm3

	// unroll 3
	vmovapd 		96(%ebx), %ymm4 // A
	vbroadcastsd	96(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm0, %ymm7, %ymm0
	vbroadcastsd	104(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm1, %ymm7, %ymm1
	vbroadcastsd	112(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm2, %ymm7, %ymm2
	vbroadcastsd	120(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm3, %ymm7, %ymm3

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
	vmovapd 		0(%ebx), %ymm4 // A
	vbroadcastsd	0(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm0, %ymm7, %ymm0
	vbroadcastsd	8(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm1, %ymm7, %ymm1
	vbroadcastsd	16(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm2, %ymm7, %ymm2
	vbroadcastsd	24(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm3, %ymm7, %ymm3

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

	.size	inner_kernel_gemm_nt_4x4_lib4, .-inner_kernel_gemm_nt_4x4_lib4
#endif





// common inner routine with file scope
//
// input arguments:
// eax  <- k
// ebx   <- A
// ecx   <- B
// edx   <- 4*sdb*sizeof(double)
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]

//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_4X4_LIB4
#else
	.p2align 4,,15
	.type inner_kernel_gemm_nn_4x4_lib4, @function
inner_kernel_gemm_nn_4x4_lib4:
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
	prefetcht0	64(%ecx, %edx, 1) // software prefetch

	// unroll 0
	vmovapd 		0(%ebx), %ymm4 // A
	vbroadcastsd	0(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm0, %ymm7, %ymm0
	vbroadcastsd	32(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm1, %ymm7, %ymm1
	vbroadcastsd	64(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm2, %ymm7, %ymm2
	vbroadcastsd	96(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm3, %ymm7, %ymm3

	// unroll 1
	vmovapd 		32(%ebx), %ymm4 // A
	vbroadcastsd	8(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm0, %ymm7, %ymm0
	vbroadcastsd	40(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm1, %ymm7, %ymm1
	vbroadcastsd	72(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm2, %ymm7, %ymm2
	vbroadcastsd	104(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm3, %ymm7, %ymm3

	// unroll 2
	vmovapd 		64(%ebx), %ymm4 // A
	vbroadcastsd	16(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm0, %ymm7, %ymm0
	vbroadcastsd	48(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm1, %ymm7, %ymm1
	vbroadcastsd	80(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm2, %ymm7, %ymm2
	vbroadcastsd	112(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm3, %ymm7, %ymm3

	// unroll 3
	vmovapd 		96(%ebx), %ymm4 // A
	vbroadcastsd	24(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm0, %ymm7, %ymm0
	vbroadcastsd	56(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm1, %ymm7, %ymm1
	vbroadcastsd	88(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm2, %ymm7, %ymm2
	vbroadcastsd	120(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm3, %ymm7, %ymm3

	subl	$ 4, %eax
	addl	$ 128, %ebx
	addl	%edx, %ecx

	cmpl	$ 4, %eax
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %eax
	jle		4f // clean1

	// unroll 0
	vmovapd 		0(%ebx), %ymm4 // A
	vbroadcastsd	0(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm0, %ymm7, %ymm0
	vbroadcastsd	32(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm1, %ymm7, %ymm1
	vbroadcastsd	64(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm2, %ymm7, %ymm2
	vbroadcastsd	96(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm3, %ymm7, %ymm3

	// unroll 1
	vmovapd 		32(%ebx), %ymm4 // A
	vbroadcastsd	8(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm0, %ymm7, %ymm0
	vbroadcastsd	40(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm1, %ymm7, %ymm1
	vbroadcastsd	72(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm2, %ymm7, %ymm2
	vbroadcastsd	104(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm3, %ymm7, %ymm3

	// unroll 2
	vmovapd 		64(%ebx), %ymm4 // A
	vbroadcastsd	16(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm0, %ymm7, %ymm0
	vbroadcastsd	48(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm1, %ymm7, %ymm1
	vbroadcastsd	80(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm2, %ymm7, %ymm2
	vbroadcastsd	112(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm3, %ymm7, %ymm3

	// unroll 3
	vmovapd 		96(%ebx), %ymm4 // A
	vbroadcastsd	24(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm0, %ymm7, %ymm0
	vbroadcastsd	56(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm1, %ymm7, %ymm1
	vbroadcastsd	88(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm2, %ymm7, %ymm2
	vbroadcastsd	120(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm3, %ymm7, %ymm3

	subl	$ 4, %eax
	addl	$ 128, %ebx
	addl	%edx, %ecx

//	cmpl	$ 3, %eax
	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %eax
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovapd 		0(%ebx), %ymm4 // A
	vbroadcastsd	0(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm0, %ymm7, %ymm0
	vbroadcastsd	32(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm1, %ymm7, %ymm1
	vbroadcastsd	64(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm2, %ymm7, %ymm2
	vbroadcastsd	96(%ecx), %ymm5 // B
	vmulpd			%ymm4, %ymm5, %ymm7
	vaddpd			%ymm3, %ymm7, %ymm3

	subl	$ 1, %eax
	addl	$ 32, %ebx
	addl	$ 8, %ecx

	cmpl	$ 0, %eax
	jg		3b // clean up loop

2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	.size	inner_kernel_gemm_nn_4x4_lib4, .-inner_kernel_gemm_nn_4x4_lib4
#endif





// common inner routine with file scope
//
// edge for B unaligned, A aligned
//
// input arguments:
// eax   <- k
// ebx   <- A
// ecx   <- B
// edx   <- bs*sdb*sizeof(double)
// esi   <- offB
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]

//
// output arguments:


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_GEMM_NN_4X4_LIB4
#else
	.p2align 4,,15
	.type inner_edge_gemm_nn_4x4_lib4, @function
inner_edge_gemm_nn_4x4_lib4:
#endif

	cmpl			$ 0, %esi				// offset==0
	jle				2f						// end

	cmpl			$ 0, %eax				// k==0
	jle				2f						// end

	movl			%esi, %edi				// load offsetB
	sall			$ 3, %edi				// offsetB*sizeof(double)
	addl			%edi, %ecx				// B+offsetB*sizeof(double)

	movl			$ 4, %edi				// load 4
	subl			%esi, %edi				// 4-offsetB
	cmpl			%eax, %edi				// k > 4-offsetB
	cmovgl			%eax, %edi				// kend=min(k,4-offsetB)

1:
	vmovapd			0(%ebx), %ymm5			// load A

	vbroadcastsd	0(%ecx), %ymm4			// B_off
	vmulpd			%ymm5, %ymm4, %ymm7		// mul
	vaddpd			%ymm7, %ymm0, %ymm0		// store

	vbroadcastsd	32(%ecx), %ymm4			// B_off+4
	vmulpd			%ymm5, %ymm4, %ymm7		// mul
	vaddpd			%ymm7, %ymm1, %ymm1		// store

	vbroadcastsd	64(%ecx), %ymm4			// B_off+8
	vmulpd			%ymm5, %ymm4, %ymm7		// mul
	vaddpd			%ymm7, %ymm2, %ymm2		// store

	vbroadcastsd	96(%ecx), %ymm4			// B_off+12
	vmulpd			%ymm5, %ymm4, %ymm7		// mul
	vaddpd			%ymm7, %ymm3, %ymm3		// store

	subl			$ 1, %eax				// k=-1
	subl			$ 1, %edi				// k_panel=-1
	addl			$ 32, %ebx				// A=+bs
	addl			$ 8, %ecx				// B=+1

	cmpl			$ 0, %edi				// if k_panel=0
	jg				1b						// loop 1

	cmpl			$ 0, %eax				// if k=0
	jle				2f						// end

	addl			%edx, %ecx				// B=Boff+sdb*bs
	subl			$ 32, %ecx				// B-=4*sizeof(double) (loop+offsetB)

2:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_edge_gemm_nn_4x4_lib4, .-inner_edge_gemm_nn_4x4_lib4
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = lower
// tran = transposed
// requires explicit inverse of diagonal
//
// input arguments:
// eax  <- E
// ebx  <- inv_diag_E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLT_INV_4X4_LIB4
#else
	.p2align 4,,15
	.type inner_edge_dtrsm_rlt_inv_4x4_lib4, @function
inner_edge_dtrsm_rlt_inv_4x4_lib4:
#endif

	vbroadcastsd	0(%ebx), %ymm4
	vmulpd			%ymm0, %ymm4, %ymm0
	vbroadcastsd	8(%eax), %ymm4
	vmulpd			%ymm0, %ymm4, %ymm7
	vsubpd			%ymm7, %ymm1, %ymm1
	vbroadcastsd	16(%eax), %ymm4
	vmulpd			%ymm0, %ymm4, %ymm7
	vsubpd			%ymm7, %ymm2, %ymm2
	vbroadcastsd	24(%eax), %ymm4
	vmulpd			%ymm0, %ymm4, %ymm7
	vsubpd			%ymm7, %ymm3, %ymm3

	vbroadcastsd	8(%ebx), %ymm4
	vmulpd			%ymm1, %ymm4, %ymm1
	vbroadcastsd	48(%eax), %ymm4
	vmulpd			%ymm1, %ymm4, %ymm7
	vsubpd			%ymm7, %ymm2, %ymm2
	vbroadcastsd	56(%eax), %ymm4
	vmulpd			%ymm1, %ymm4, %ymm7
	vsubpd			%ymm7, %ymm3, %ymm3

	vbroadcastsd	16(%ebx), %ymm4
	vmulpd			%ymm2, %ymm4, %ymm2
	vbroadcastsd	88(%eax), %ymm4
	vmulpd			%ymm2, %ymm4, %ymm7
	vsubpd			%ymm7, %ymm3, %ymm3

	vbroadcastsd	24(%ebx), %ymm4
	vmulpd			%ymm3, %ymm4, %ymm3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_edge_dtrsm_rlt_inv_4x4_lib4, .-inner_edge_dtrsm_rlt_inv_4x4_lib4
#endif





// common inner routine with file scope
//
// blend for generic alpha and beta
//
// input arguments:
// eax   <- alpha
// ebx   <- beta
// ecx   <- C
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
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
	vbroadcastsd	0(%eax), %ymm7

	vmulpd		%ymm0, %ymm7, %ymm0
	vmulpd		%ymm1, %ymm7, %ymm1
	vmulpd		%ymm2, %ymm7, %ymm2
	vmulpd		%ymm3, %ymm7, %ymm3

	// beta
	vbroadcastsd	0(%ebx), %ymm6

	vxorpd		%ymm7, %ymm7, %ymm7 // 0.0

	vucomisd	%xmm7, %xmm6 // beta==0.0 ?
	je			0f // end

	vmovapd		0(%ecx), %ymm7
	vmulpd		%ymm7, %ymm6, %ymm7
	vaddpd		%ymm0, %ymm7, %ymm0
	vmovapd		32(%ecx), %ymm7
	vmulpd		%ymm7, %ymm6, %ymm7
	vaddpd		%ymm1, %ymm7, %ymm1
	vmovapd		64(%ecx), %ymm7
	vmulpd		%ymm7, %ymm6, %ymm7
	vaddpd		%ymm2, %ymm7, %ymm2
	vmovapd		96(%ecx), %ymm7
	vmulpd		%ymm7, %ymm6, %ymm7
	vaddpd		%ymm3, %ymm7, %ymm3

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_scale_ab_4x4_lib4, .-inner_scale_ab_4x4_lib4
#endif





// common inner routine with file scope
//
// blender_loader for alpha = 1.0 and beta
//
// input arguments:
// eax   <- beta
// ebx   <- C
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M1B_4X4_LIB4
#else
	.p2align 4,,15
	.type inner_scale_m1b_4x4_lib4, @function
inner_scale_m1b_4x4_lib4:
#endif

	// beta
	vbroadcastsd	0(%eax), %ymm6

	vxorpd		%ymm7, %ymm7, %ymm7 // 0.0

	vucomisd	%xmm7, %xmm6 // beta==0.0 ?
	je			0f // end

	vmovapd		0(%ebx), %ymm7
	vmulpd		%ymm7, %ymm6, %ymm7
	vsubpd		%ymm0, %ymm7, %ymm0
	vmovapd		32(%ebx), %ymm7
	vmulpd		%ymm7, %ymm6, %ymm7
	vsubpd		%ymm1, %ymm7, %ymm1
	vmovapd		64(%ebx), %ymm7
	vmulpd		%ymm7, %ymm6, %ymm7
	vsubpd		%ymm2, %ymm7, %ymm2
	vmovapd		96(%ebx), %ymm7
	vmulpd		%ymm7, %ymm6, %ymm7
	vsubpd		%ymm3, %ymm7, %ymm3

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_scale_m1b_4x4_lib4, .-inner_scale_m1b_4x4_lib4
#endif
#endif





// common inner routine with file scope
//
// blender_loader for alpha = 1.0 and beta = 1.0
//
// input arguments:
// eax   <- C
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M11_4X4_LIB4
#else
	.p2align 4,,15
	.type inner_scale_m11_4x4_lib4, @function
inner_scale_m11_4x4_lib4:
#endif

	vmovapd		0(%eax), %ymm7
	vsubpd		%ymm0, %ymm7, %ymm0
	vmovapd		32(%eax), %ymm7
	vsubpd		%ymm1, %ymm7, %ymm1
	vmovapd		64(%eax), %ymm7
	vsubpd		%ymm2, %ymm7, %ymm2
	vmovapd		96(%eax), %ymm7
	vsubpd		%ymm3, %ymm7, %ymm3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_scale_m11_4x4_lib4, .-inner_scale_m11_4x4_lib4
#endif
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// eax  <- D
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X4_LIB4
#else
	.align 16
	.type inner_store_4x4_lib4, @function
inner_store_4x4_lib4:
#endif

	vmovapd	%ymm0,  0(%eax)
	vmovapd %ymm1, 32(%eax)
	vmovapd %ymm2, 64(%eax)
	vmovapd %ymm3, 96(%eax)

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
// eax   <- D
// ebx   <- km
// ecx   <- kn
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d03 d12 d21 d30]
// ymm3  <- [d02 d13 d20 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X4_VS_LIB4
#else
	.align 16
	.type inner_store_4x4_vs_lib4, @function
inner_store_4x4_vs_lib4:
#endif

	vcvtsi2sd	%ebx, %xmm7, %xmm7
	vmovupd		.LC02, %ymm6
	vmovddup	%xmm7, %xmm7
	vinsertf128	$ 1, %xmm7, %ymm7, %ymm7
	vsubpd		%ymm7, %ymm6, %ymm7

	cmpl		$ 2, %ecx
	vmaskmovpd	%ymm0, %ymm7,  0(%eax)
	jl			0f // end
	cmpl		$ 3, %ecx
	vmaskmovpd	%ymm1, %ymm7, 32(%eax)
	jl			0f // end
	vmaskmovpd	%ymm2, %ymm7, 64(%eax)
	je			0f // end
	vmaskmovpd	%ymm3, %ymm7, 96(%eax)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_store_4x4_vs_lib4, .-inner_store_4x4_vs_lib4
#endif





//                               1      2              3          4          5             6          7
// void kernel_dgemm_nt_4x4_lib4(int k, double *alpha, double *A, double *B, double *beta, double *C, double *D);

	.align 16
	.globl kernel_dgemm_nt_4x4_lib4
	.type kernel_dgemm_nt_4x4_lib4, @function
kernel_dgemm_nt_4x4_lib4:

	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3


	// call inner gemm kernel nt

	movl	ARG1, %eax // k
	movl	ARG3, %ebx  // A
	movl	ARG4, %ecx  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	call inner_kernel_gemm_nt_4x4_lib4
#endif


	// call inner blend scale

	movl	ARG2, %eax // alpha
	movl	ARG5, %ebx // beta
	movl	ARG6, %ecx   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	call inner_scale_ab_4x4_lib4
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

	.size	kernel_dgemm_nt_4x4_lib4, .-kernel_dgemm_nt_4x4_lib4





//                                  1      2              3          4          5             6          7          8       9
// void kernel_dgemm_nt_4x4_vs_lib4(int k, double *alpha, double *A, double *B, double *beta, double *C, double *D, int m1, int n1);

	.align 16
	.globl kernel_dgemm_nt_4x4_vs_lib4
	.type kernel_dgemm_nt_4x4_vs_lib4, @function
kernel_dgemm_nt_4x4_vs_lib4:

	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3


	// call inner gemm kernel nt

	movl	ARG1, %eax // k
	movl	ARG3, %ebx  // A
	movl	ARG4, %ecx  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	call inner_kernel_gemm_nt_4x4_lib4
#endif


	// call inner blend scale

	movl	ARG2, %eax // alpha
	movl	ARG5, %ebx // beta
	movl	ARG6, %ecx   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	call inner_scale_ab_4x4_lib4
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

	.size	kernel_dgemm_nt_4x4_vs_lib4, .-kernel_dgemm_nt_4x4_vs_lib4





//                               1      2              3          4            5          6        7             8          9
// void kernel_dgemm_nn_4x4_lib4(int k, double *alpha, double *A, int offsetB, double *B, int sdb, double *beta, double *C, double *D);

	.align 16
	.globl kernel_dgemm_nn_4x4_lib4
	.type kernel_dgemm_nn_4x4_lib4, @function
kernel_dgemm_nn_4x4_lib4:

	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3


	// call inner gemm kernel nn

	movl	ARG1, %eax // k
	movl	ARG3, %ebx  // A
	movl	ARG5, %ecx  // B
	movl	ARG6, %edx // sdb
	sall	$ 5, %edx // 4*sdb*sizeof(double)
	movl	ARG4, %esi // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMM_NN_4X4_LIB4
#else
	call inner_edge_gemm_nn_4x4_lib4
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X4_LIB4
#else
	call inner_kernel_gemm_nn_4x4_lib4
#endif


	// call inner blend

	movl	ARG2, %eax // alpha
	movl	ARG7, %ebx // beta
	movl	ARG8, %ecx   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	call inner_scale_ab_4x4_lib4
#endif


	// store n

	movl	ARG9, %eax // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
	call inner_store_4x4_lib4
#endif


	EPILOGUE

	ret

	.size	kernel_dgemm_nn_4x4_lib4, .-kernel_dgemm_nn_4x4_lib4





//                                  1      2              3          4            5          6        7             8          9          10      11
// void kernel_dgemm_nn_4x4_vs_lib4(int k, double *alpha, double *A, int offsetB, double *B, int sdb, double *beta, double *C, double *D, int m1, int n1);

	.align 16
	.globl kernel_dgemm_nn_4x4_vs_lib4
	.type kernel_dgemm_nn_4x4_vs_lib4, @function
kernel_dgemm_nn_4x4_vs_lib4:

	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3


	// call inner gemm kernel nn

	movl	ARG1, %eax // k
	movl	ARG3, %ebx  // A
	movl	ARG5, %ecx  // B
	movl	ARG6, %edx // sdb
	sall	$ 5, %edx // 4*sdb*sizeof(double)
	movl	ARG4, %esi // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMM_NN_4X4_LIB4
#else
	call inner_edge_gemm_nn_4x4_lib4
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X4_LIB4
#else
	call inner_kernel_gemm_nn_4x4_lib4
#endif


	// call inner blend

	movl	ARG2, %eax // alpha
	movl	ARG7, %ebx // beta
	movl	ARG8, %ecx   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	call inner_scale_ab_4x4_lib4
#endif


	// store n

	movl	ARG9, %eax // D
	movl	ARG10, %ebx // m1
	movl	ARG11, %ecx // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	call inner_store_4x4_vs_lib4
#endif


	EPILOGUE

	ret

	.size	kernel_dgemm_nn_4x4_vs_lib4, .-kernel_dgemm_nn_4x4_vs_lib4





//                                      1      2          3          4             5          6          7          8
// void kernel_dtrsm_nt_rl_inv_4x4_lib4(int k, double *A, double *B, double *beta, double *C, double *D, double *E, double *inv_diag_E);

	.align 16
	.globl kernel_dtrsm_nt_rl_inv_4x4_lib4
	.type kernel_dtrsm_nt_rl_inv_4x4_lib4, @function
kernel_dtrsm_nt_rl_inv_4x4_lib4:

	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3


	// call inner gemm kernel nt

	movl	ARG1, %eax // kmax
	movl	ARG2, %ebx // A
	movl	ARG3, %ecx // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	call inner_kernel_gemm_nt_4x4_lib4
#endif


	// call inner blender_loader nn

	movl	ARG4, %eax // beta
	movl	ARG5, %ebx // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB4
#else
	call inner_scale_m1b_4x4_lib4
#endif


	// solve

	movl	ARG7, %eax  // E
	movl	ARG8, %ebx  // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_4X4_LIB4
#else
	call inner_edge_dtrsm_rlt_inv_4x4_lib4
#endif


	// store

	movl	ARG6, %eax // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
	call inner_store_4x4_lib4
#endif


	EPILOGUE

	ret

	.size	kernel_dtrsm_nt_rl_inv_4x4_lib4, .-kernel_dtrsm_nt_rl_inv_4x4_lib4





//                                         1      2          3          4             5          6          7          8                   9       10
// void kernel_dtrsm_nt_rl_inv_4x4_vs_lib4(int k, double *A, double *B, double *beta, double *C, double *D, double *E, double *inv_diag_E, int m1, int n1);

	.align 16
	.globl kernel_dtrsm_nt_rl_inv_4x4_vs_lib4
	.type kernel_dtrsm_nt_rl_inv_4x4_vs_lib4, @function
kernel_dtrsm_nt_rl_inv_4x4_vs_lib4:

	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3


	// call inner gemm kernel nt

	movl	ARG1, %eax // kmax
	movl	ARG2, %ebx // A
	movl	ARG3, %ecx // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	call inner_kernel_gemm_nt_4x4_lib4
#endif


	// call inner blender_loader nn

	movl	ARG4, %eax // beta
	movl	ARG5, %ebx // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB4
#else
	call inner_scale_m1b_4x4_lib4
#endif


	// solve

	movl	ARG7, %eax  // E
	movl	ARG8, %ebx  // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_4X4_LIB4
#else
	call inner_edge_dtrsm_rlt_inv_4x4_lib4
#endif


	// store

	movl	ARG6, %eax // D
	movl	ARG9, %ebx // m1
	movl	ARG10, %ecx // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	call inner_store_4x4_vs_lib4
#endif


	EPILOGUE

	ret

	.size	kernel_dtrsm_nt_rl_inv_4x4_vs_lib4, .-kernel_dtrsm_nt_rl_inv_4x4_vs_lib4





//#if defined(BLAS_API)
#if ( defined(BLAS_API) | ( defined(LA_HIGH_PERFORMANCE) & defined(MF_COLMAJ) ) )

#include "kernel_dgemm_4x4_lib.S"

#endif





	// read-only data
	.section	.rodata.cst32,"aM",@progbits,32

	.align 32
.LC00: // { -1 -1 -1 1 }
	.quad	-1
	.quad	-1
	.quad	-1
	.quad	1

	.align 32
.LC01: // { -1 -1 -1 -1 }
	.quad	-1
	.quad	-1
	.quad	-1
	.quad	-1

	.align 32
.LC02: // { 3.5 2.5 1.5 0.5 }
	.long	0
	.long	1071644672
	.long	0
	.long	1073217536
	.long	0
	.long	1074003968
	.long	0
	.long	1074528256

	.align 32
.LC03: // { 7.5 6.5 5.5 4.5 }
	.long	0
	.long	1074921472
	.long	0
	.long	1075183616
	.long	0
	.long	1075445760
	.long	0
	.long	1075707904

	.align 32
.LC04: // { 1.0 1.0 1.0 1.0 }
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248

	.align 32
.LC05: // { 1.0 1.0 1.0 -1.0 }
	.long	0
	.long	-1074790400
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248

	.align 32
.LC06: // { 1.0 1.0 -1.0 -1.0 }
	.long	0
	.long	-1074790400
	.long	0
	.long	-1074790400
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248

	.align 32
.LC07: // { 1.0 -1.0 -1.0 -1.0 }
	.long	0
	.long	-1074790400
	.long	0
	.long	-1074790400
	.long	0
	.long	-1074790400
	.long	0
	.long	1072693248

	.align 32
.LC08: // { -1.0 -1.0 -1.0 1.0 }
	.long	0
	.long	1072693248
	.long	0
	.long	-1074790400
	.long	0
	.long	-1074790400
	.long	0
	.long	-1074790400

	.align 32
.LC09: // { -1.0 -1.0 1.0 1.0 }
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248
	.long	0
	.long	-1074790400
	.long	0
	.long	-1074790400

	.align 32
.LC10: // { -1.0 1.0 1.0 1.0 }
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248
	.long	0
	.long	-1074790400



	.section	.note.GNU-stack,"",@progbits

