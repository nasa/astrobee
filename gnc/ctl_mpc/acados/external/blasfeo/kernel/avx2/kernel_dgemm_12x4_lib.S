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
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// r14   <- A_p
// r15   <- B_p
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d03 d12 d21 d30]
// ymm3  <- [d02 d13 d20 d31]
// ymm4  <- [d40 d51 d62 d73]
// ymm5  <- [d41 d50 d63 d72]
// ymm6  <- [d43 d52 d61 d70]
// ymm7  <- [d42 d53 d60 d71]
// ymm8  <- [d80 d91 da2 db3]
// ymm9  <- [d81 d90 da3 db2]
// ymm10 <- [d83 d92 da1 db0]
// ymm11 <- [d82 d93 da0 db1]

//
// output arguments:
// r10d  <- 0
// r11   <- A+4*k*sizeof(double)
// r12   <- 4*sda*sizeof(double)
// r13   <- B+4*k*sizeof(double)
// r14   <- A_p
// r15   <- B_p
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d03 d12 d21 d30]
// ymm3  <- [d02 d13 d20 d31]
// ymm4  <- [d40 d51 d62 d73]
// ymm5  <- [d41 d50 d63 d72]
// ymm6  <- [d43 d52 d61 d70]
// ymm7  <- [d42 d53 d60 d71]
// ymm8  <- [d80 d91 da2 db3]
// ymm9  <- [d81 d90 da3 db2]
// ymm10 <- [d83 d92 da1 db0]
// ymm11 <- [d82 d93 da0 db1]



#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_12X4_P0_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_12x4_p0_lib4)
#endif

// broadcast scheme

	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch

	// preload
	vmovapd 0(%r11), %ymm13 // A0[0]
	vmovapd 0(%r11, %r12), %ymm14 // A1[0]
	vmovapd 0(%r11, %r12, 2), %ymm15 // A2[0]

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	cmpl	$ 8, %r10d
	jle		5f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop


	// unroll 0
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0	2*128(%r11) // prefetch A
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	prefetcht0	2*128(%r11, %r12) // prefetch A
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0	2*128(%r11, %r12, 2) // prefetch A
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	prefetcht0	2*128(%r13) // prefetch B
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	subl	$ 4, %r10d

	vbroadcastsd	16(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	24(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			32(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			32(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			32(%r11, %r12, 2), %ymm15 // A1

	// unroll 1
	vbroadcastsd	32(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	40(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	48(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	56(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			64(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			64(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			64(%r11, %r12, 2), %ymm15 // A1

	// unroll 2
	vbroadcastsd	64(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0	2*128+64(%r11) // prefetch A
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	prefetcht0	2*128+64(%r11, %r12) // prefetch A
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	72(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0	2*128+64(%r11, %r12, 2) // prefetch A
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	prefetcht0	2*128+64(%r13) // prefetch B
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	80(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	88(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			96(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			96(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			96(%r11, %r12, 2), %ymm15 // A1

	// unroll 3
	vbroadcastsd	96(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	addq	$ 128, %r11

	vbroadcastsd	104(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	112(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	120(%r13), %ymm12
	addq	$ 128, %r13
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			0(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			0(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			0(%r11, %r12, 2), %ymm15 // A1


//	cmpl	$ 4, %r10d
	cmpl	$ 8, %r10d
	jg		1b // main loop



5: // consider clean8-up


	// unroll 0
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0	0(%r14) // prefetch A_p
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	prefetcht0	0(%r14, %r12) // prefetch A_p
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0	0(%r14, %r12, 2) // prefetch A_p
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	prefetcht0	0(%r15) // prefetch B
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	subl	$ 4, %r10d

	vbroadcastsd	16(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	24(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			32(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			32(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			32(%r11, %r12, 2), %ymm15 // A1

	// unroll 1
	vbroadcastsd	32(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	40(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	48(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	56(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			64(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			64(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			64(%r11, %r12, 2), %ymm15 // A1

	// unroll 2
	vbroadcastsd	64(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0	64(%r14) // prefetch A_p
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	prefetcht0	64(%r14, %r12) // prefetch A_p
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	72(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0	64(%r14, %r12, 2) // prefetch A_p
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	prefetcht0	64(%r15) // prefetch B
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	80(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	88(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			96(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			96(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			96(%r11, %r12, 2), %ymm15 // A1

	// unroll 3
	vbroadcastsd	96(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	addq	$ 128, %r11

	vbroadcastsd	104(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	112(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	120(%r13), %ymm12
	addq	$ 128, %r13
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			0(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			0(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			0(%r11, %r12, 2), %ymm15 // A1



0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0	128(%r14) // prefetch A_p
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	prefetcht0	128(%r14, %r12) // prefetch A_p
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0	128(%r14, %r12, 2) // prefetch A_p
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	prefetcht0	128(%r15) // prefetch B
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	subl	$ 4, %r10d

	vbroadcastsd	16(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	24(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			32(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			32(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			32(%r11, %r12, 2), %ymm15 // A1

	// unroll 1
	vbroadcastsd	32(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	40(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	48(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	56(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			64(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			64(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			64(%r11, %r12, 2), %ymm15 // A1

	// unroll 2
	vbroadcastsd	64(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0	128+64(%r14) // prefetch A_p
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	prefetcht0	128+64(%r14, %r12) // prefetch A_p
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	72(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0	128+64(%r14, %r12, 2) // prefetch A_p
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	prefetcht0	128+64(%r15) // prefetch B
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	80(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	88(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			96(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			96(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			96(%r11, %r12, 2), %ymm15 // A1

	// unroll 3
	vbroadcastsd	96(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	addq	$ 128, %r11

	vbroadcastsd	104(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	112(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	120(%r13), %ymm12
	addq	$ 128, %r13
	vfmadd231pd		%ymm13, %ymm12, %ymm3
//	vmovapd			0(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
//	vmovapd			0(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
//	vmovapd			0(%r11, %r12, 2), %ymm15 // A1


	jmp		2f


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	vmovapd			0(%r11), %ymm13 // A0[0]
	vmovapd 		0(%r11, %r12), %ymm14 // A1[0]
	vmovapd 		0(%r11, %r12, 2), %ymm15 // A2[0]
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	addq	$ 32, %r11

	vbroadcastsd	16(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	subl	$ 1, %r10d

	vbroadcastsd	24(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

FUN_END(inner_kernel_dgemm_nt_12x4_p0_lib4)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// r14   <- A_p
// r15   <- B_p
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d03 d12 d21 d30]
// ymm3  <- [d02 d13 d20 d31]
// ymm4  <- [d40 d51 d62 d73]
// ymm5  <- [d41 d50 d63 d72]
// ymm6  <- [d43 d52 d61 d70]
// ymm7  <- [d42 d53 d60 d71]
// ymm8  <- [d80 d91 da2 db3]
// ymm9  <- [d81 d90 da3 db2]
// ymm10 <- [d83 d92 da1 db0]
// ymm11 <- [d82 d93 da0 db1]

//
// output arguments:
// r10d  <- 0
// r11   <- A+4*k*sizeof(double)
// r12   <- 4*sda*sizeof(double)
// r13   <- B+4*k*sizeof(double)
// r14   <- A_p
// r15   <- B_p
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d03 d12 d21 d30]
// ymm3  <- [d02 d13 d20 d31]
// ymm4  <- [d40 d51 d62 d73]
// ymm5  <- [d41 d50 d63 d72]
// ymm6  <- [d43 d52 d61 d70]
// ymm7  <- [d42 d53 d60 d71]
// ymm8  <- [d80 d91 da2 db3]
// ymm9  <- [d81 d90 da3 db2]
// ymm10 <- [d83 d92 da1 db0]
// ymm11 <- [d82 d93 da0 db1]



#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_12X4_PL_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_12x4_pl_lib4)
#endif

// broadcast scheme

	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch

	// preload
	vmovapd 0(%r11), %ymm13 // A0[0]
	vmovapd 0(%r11, %r12), %ymm14 // A1[0]
	vmovapd 0(%r11, %r12, 2), %ymm15 // A2[0]

	movq	%r14, %rax

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	cmpl	$ 8, %r10d
	jle		5f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop


	// unroll 0
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0	2*128(%r11) // prefetch A
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	prefetcht0	2*128(%r11, %r12) // prefetch A
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0	2*128(%r11, %r12, 2) // prefetch A
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	prefetcht0	2*128(%r13) // prefetch B
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	subl	$ 4, %r10d

	vbroadcastsd	16(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	24(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			32(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			32(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			32(%r11, %r12, 2), %ymm15 // A1

	// unroll 1
	vbroadcastsd	32(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht2	0(%rax) // prefetch A_p
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	prefetcht2	0(%rax, %r12) // prefetch A_p
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	40(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht2	0(%rax, %r12, 2) // prefetch A_p
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	prefetcht2	64(%rax) // prefetch A_p
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	48(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	prefetcht2	64(%rax, %r12) // prefetch A_p
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	prefetcht2	64(%rax, %r12, 2) // prefetch A_p
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	addq	$ 128, %rax

	vbroadcastsd	56(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			64(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			64(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			64(%r11, %r12, 2), %ymm15 // A1

	// unroll 2
	vbroadcastsd	64(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0	2*128+64(%r11) // prefetch A
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	prefetcht0	2*128+64(%r11, %r12) // prefetch A
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	72(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0	2*128+64(%r11, %r12, 2) // prefetch A
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	prefetcht0	2*128+64(%r13) // prefetch B
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	80(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	88(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			96(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			96(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			96(%r11, %r12, 2), %ymm15 // A1

	// unroll 3
	vbroadcastsd	96(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	addq	$ 128, %r11

	vbroadcastsd	104(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	112(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	120(%r13), %ymm12
	addq	$ 128, %r13
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			0(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			0(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			0(%r11, %r12, 2), %ymm15 // A1


//	cmpl	$ 4, %r10d
	cmpl	$ 8, %r10d
	jg		1b // main loop



5: // consider clean8-up


	// unroll 0
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0	0(%r14) // prefetch A_p
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	prefetcht0	0(%r14, %r12) // prefetch A_p
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0	0(%r14, %r12, 2) // prefetch A_p
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	prefetcht0	0(%r15) // prefetch B
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	subl	$ 4, %r10d

	vbroadcastsd	16(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	24(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			32(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			32(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			32(%r11, %r12, 2), %ymm15 // A1

	// unroll 1
	vbroadcastsd	32(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht2	0(%rax) // prefetch A_p
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	prefetcht2	0(%rax, %r12) // prefetch A_p
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	40(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht2	0(%rax, %r12, 2) // prefetch A_p
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	prefetcht2	64(%rax) // prefetch A_p
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	48(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	prefetcht2	64(%rax, %r12) // prefetch A_p
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	prefetcht2	64(%rax, %r12, 2) // prefetch A_p
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	addq	$ 128, %rax

	vbroadcastsd	56(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			64(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			64(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			64(%r11, %r12, 2), %ymm15 // A1

	// unroll 2
	vbroadcastsd	64(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0	64(%r14) // prefetch A_p
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	prefetcht0	64(%r14, %r12) // prefetch A_p
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	72(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0	64(%r14, %r12, 2) // prefetch A_p
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	prefetcht0	64(%r15) // prefetch B
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	80(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	88(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			96(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			96(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			96(%r11, %r12, 2), %ymm15 // A1

	// unroll 3
	vbroadcastsd	96(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	addq	$ 128, %r11

	vbroadcastsd	104(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	112(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	120(%r13), %ymm12
	addq	$ 128, %r13
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			0(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			0(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			0(%r11, %r12, 2), %ymm15 // A1



0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0	128(%r14) // prefetch A_p
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	prefetcht0	128(%r14, %r12) // prefetch A_p
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0	128(%r14, %r12, 2) // prefetch A_p
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	prefetcht0	128(%r15) // prefetch B
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	subl	$ 4, %r10d

	vbroadcastsd	16(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	24(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			32(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			32(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			32(%r11, %r12, 2), %ymm15 // A1

	// unroll 1
	vbroadcastsd	32(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht2	0(%rax) // prefetch A_p
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	prefetcht2	0(%rax, %r12) // prefetch A_p
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	40(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht2	0(%rax, %r12, 2) // prefetch A_p
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	prefetcht2	64(%rax) // prefetch A_p
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	48(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	prefetcht2	64(%rax, %r12) // prefetch A_p
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	prefetcht2	64(%rax, %r12, 2) // prefetch A_p
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	addq	$ 128, %rax

	vbroadcastsd	56(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			64(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			64(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			64(%r11, %r12, 2), %ymm15 // A1

	// unroll 2
	vbroadcastsd	64(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0	128+64(%r14) // prefetch A_p
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	prefetcht0	128+64(%r14, %r12) // prefetch A_p
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	72(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0	128+64(%r14, %r12, 2) // prefetch A_p
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	prefetcht0	128+64(%r15) // prefetch B
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	80(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	88(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			96(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			96(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			96(%r11, %r12, 2), %ymm15 // A1

	// unroll 3
	vbroadcastsd	96(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	addq	$ 128, %r11

	vbroadcastsd	104(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	112(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	120(%r13), %ymm12
	addq	$ 128, %r13
	vfmadd231pd		%ymm13, %ymm12, %ymm3
//	vmovapd			0(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
//	vmovapd			0(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
//	vmovapd			0(%r11, %r12, 2), %ymm15 // A1


	jmp		2f


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	vmovapd			0(%r11), %ymm13 // A0[0]
	vmovapd 		0(%r11, %r12), %ymm14 // A1[0]
	vmovapd 		0(%r11, %r12, 2), %ymm15 // A2[0]
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	addq	$ 32, %r11

	vbroadcastsd	16(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	subl	$ 1, %r10d

	vbroadcastsd	24(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

FUN_END(inner_kernel_dgemm_nt_12x4_pl_lib4)
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
	.macro INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_12x4_lib4c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	prefetcht0		31(%r13, %r14, 8)
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	prefetcht0		31(%r13, %r14, 8)
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vmovupd			64(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	prefetcht0		31(%r13, %r14, 8)
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	subl	$ 4, %r10d
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vmovupd			96(%r11, %r12), %ymm14 // A
	vmovupd			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	prefetcht0		31(%r13, %r14, 8)
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	addq	$ 128, %r11
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	addq	%r14, %r13
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11


	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vmovupd			64(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vmovupd			96(%r11, %r12), %ymm14 // A
	vmovupd			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
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
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
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

	FUN_END(inner_kernel_dgemm_nt_12x4_lib4c)
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
	.macro INNER_KERNEL_DGEMM_NT_12X3_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_12x3_lib4c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	addq	%r14, %r13

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	addq	%r14, %r13

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vmovupd			64(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	addq	%r14, %r13

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vmovupd			96(%r11, %r12), %ymm14 // A
	vmovupd			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
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
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	addq	%r14, %r13

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	addq	%r14, %r13

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vmovupd			64(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	addq	%r14, %r13

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vmovupd			96(%r11, %r12), %ymm14 // A
	vmovupd			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
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
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
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

	FUN_END(inner_kernel_dgemm_nt_12x3_lib4c)
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
	.macro INNER_KERNEL_DGEMM_NT_12X2_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_12x2_lib4c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	addq	%r14, %r13

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	addq	%r14, %r13

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vmovupd			64(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	addq	%r14, %r13

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vmovupd			96(%r11, %r12), %ymm14 // A
	vmovupd			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
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
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	addq	%r14, %r13

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	addq	%r14, %r13

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vmovupd			64(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	addq	%r14, %r13

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vmovupd			96(%r11, %r12), %ymm14 // A
	vmovupd			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
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
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
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

	FUN_END(inner_kernel_dgemm_nt_12x2_lib4c)
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
	.macro INNER_KERNEL_DGEMM_NT_12X1_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_12x1_lib4c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	addq	%r14, %r13

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	addq	%r14, %r13

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vmovupd			64(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	addq	%r14, %r13

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vmovupd			96(%r11, %r12), %ymm14 // A
	vmovupd			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
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
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	addq	%r14, %r13

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	addq	%r14, %r13

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vmovupd			64(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	addq	%r14, %r13

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vmovupd			96(%r11, %r12), %ymm14 // A
	vmovupd			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
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
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
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

	FUN_END(inner_kernel_dgemm_nt_12x1_lib4c)
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
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- [840 d90 da0 db0]
// ymm9  <- [841 d91 da1 db1]
// ymm10 <- [842 d92 da2 db2]
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_12x4_lib4c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	leaq	0(%r14, %r14, 2), %r15

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop


	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0		1*32(%r13)
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		1*32(%r13, %r14)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	prefetcht0		1*32(%r13, %r14, 2)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	0(%r13, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		1*32(%r13, %r15)
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	8(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	subl	$ 4, %r10d
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	8(%r13, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vmovupd			64(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	16(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	16(%r13, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vmovupd			96(%r11, %r12), %ymm14 // A
	vmovupd			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	addq	$ 128, %r11
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	24(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	24(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	addq	$ 32, %r13
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11


	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	0(%r13, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	8(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	8(%r13, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vmovupd			64(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	16(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	16(%r13, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vmovupd			96(%r11, %r12), %ymm14 // A
	vmovupd			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	24(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	24(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl	$ 4, %r10d
	addq	$ 32, %r13
	addq	$ 128, %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	0(%r13, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl	$ 1, %r10d
	addq	$ 8, %r13
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_12x4_lib4c)
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
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- [840 d90 da0 db0]
// ymm9  <- [841 d91 da1 db1]
// ymm10 <- [842 d92 da2 db2]
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_12X3_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_12x3_lib4c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	movq	%r13, %r15
	addq	%r14, %r15
	addq	%r14, %r15 // B+2*ldb

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
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	0(%r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	8(%r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vmovupd			64(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	16(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vmovupd			96(%r11, %r12), %ymm14 // A
	vmovupd			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	24(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	24(%r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	subl	$ 4, %r10d
	addq	$ 32, %r13
	addq	$ 32, %r15
	addq	$ 128, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	0(%r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	8(%r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vmovupd			64(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	16(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vmovupd			96(%r11, %r12), %ymm14 // A
	vmovupd			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	24(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	24(%r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	subl	$ 4, %r10d
	addq	$ 32, %r13
	addq	$ 32, %r15
	addq	$ 128, %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	0(%r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	subl	$ 1, %r10d
	addq	$ 8, %r13
	addq	$ 8, %r15
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_12x3_lib4c)
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
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- [840 d90 da0 db0]
// ymm9  <- [841 d91 da1 db1]
// ymm10 <- [842 d92 da2 db2]
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_12X2_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_12x2_lib4c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	movq	%r13, %r15
	addq	%r14, %r15
	addq	%r14, %r15 // B+2*ldb

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
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vmovupd			64(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	16(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vmovupd			96(%r11, %r12), %ymm14 // A
	vmovupd			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	24(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	subl	$ 4, %r10d
	addq	$ 32, %r13
	addq	$ 32, %r15
	addq	$ 128, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vmovupd			64(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	16(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vmovupd			96(%r11, %r12), %ymm14 // A
	vmovupd			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	24(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	subl	$ 4, %r10d
	addq	$ 32, %r13
	addq	$ 32, %r15
	addq	$ 128, %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	subl	$ 1, %r10d
	addq	$ 8, %r13
	addq	$ 8, %r15
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_12x2_lib4c)
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
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- [840 d90 da0 db0]
// ymm9  <- [841 d91 da1 db1]
// ymm10 <- [842 d92 da2 db2]
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_12X1_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_12x1_lib4c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	movq	%r13, %r15
	addq	%r14, %r15
	addq	%r14, %r15 // B+2*ldb

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
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vmovupd			64(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vmovupd			96(%r11, %r12), %ymm14 // A
	vmovupd			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	subl	$ 4, %r10d
	addq	$ 32, %r13
	addq	$ 32, %r15
	addq	$ 128, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vmovupd			64(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vmovupd			96(%r11, %r12), %ymm14 // A
	vmovupd			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	subl	$ 4, %r10d
	addq	$ 32, %r13
	addq	$ 32, %r15
	addq	$ 128, %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	subl	$ 1, %r10d
	addq	$ 8, %r13
	addq	$ 8, %r15
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_12x1_lib4c)
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
	.macro INNER_KERNEL_DGEMM_NT_4X12_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_4x12_lib4c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	// preload

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	prefetcht0		0(%r12, %r13, 8)
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	72(%r12), %ymm12 // B
	prefetcht0		64(%r12, %r13, 8)
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	80(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	88(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11
	addq	%r13, %r12

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	prefetcht0		0(%r12, %r13, 8)
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	72(%r12), %ymm12 // B
	prefetcht0		64(%r12, %r13, 8)
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	80(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	88(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11
	addq	%r13, %r12

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	prefetcht0		0(%r12, %r13, 8)
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	72(%r12), %ymm12 // B
	prefetcht0		64(%r12, %r13, 8)
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	80(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	88(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11
	addq	%r13, %r12

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	prefetcht0		0(%r12, %r13, 8)
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	72(%r12), %ymm12 // B
	prefetcht0		64(%r12, %r13, 8)
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	80(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	88(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11
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
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	72(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	80(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	88(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11
	addq	%r13, %r12

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	72(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	80(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	88(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11
	addq	%r13, %r12

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	72(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	80(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	88(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11
	addq	%r13, %r12

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	72(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	80(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	88(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11
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
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	72(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	80(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	88(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11
	addq	%r13, %r12

	subl	$ 1, %r10d
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_4x12_lib4c)
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
	.macro INNER_KERNEL_DGEMM_NT_4X11_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_4x11_lib4c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	// preload

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	72(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	80(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	addq	%r13, %r12

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	72(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	80(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	addq	%r13, %r12

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	72(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	80(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	addq	%r13, %r12

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	72(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	80(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
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
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	72(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	80(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	addq	%r13, %r12

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	72(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	80(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	addq	%r13, %r12

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	72(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	80(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	addq	%r13, %r12

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	72(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	80(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
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
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	72(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	80(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	addq	%r13, %r12

	subl	$ 1, %r10d
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_4x11_lib4c)
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
	.macro INNER_KERNEL_DGEMM_NT_4X10_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_4x10_lib4c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	// preload

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	72(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	addq	%r13, %r12

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	72(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	addq	%r13, %r12

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	72(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	addq	%r13, %r12

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	72(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
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
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	72(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	addq	%r13, %r12

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	72(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	addq	%r13, %r12

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	72(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	addq	%r13, %r12

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	72(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
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
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	72(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	addq	%r13, %r12

	subl	$ 1, %r10d
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_4x10_lib4c)
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
	.macro INNER_KERNEL_DGEMM_NT_4X9_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_4x9_lib4c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	// preload

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	addq	%r13, %r12

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	addq	%r13, %r12

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	addq	%r13, %r12

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
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
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	addq	%r13, %r12

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	addq	%r13, %r12

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	addq	%r13, %r12

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
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
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	addq	%r13, %r12

	subl	$ 1, %r10d
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_4x9_lib4c)
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
	.macro INNER_KERNEL_DGEMM_NN_4X12_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_4x12_lib4c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	leaq	0(%r13, %r13, 2), %r15
	leaq	0(%r12, %r13, 4), %rax
	leaq	0(%r12, %r13, 8), %rbx

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
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	0(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	0(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	0(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	0(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	0(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	0(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	0(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	0(%rbx, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	0(%rbx, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	0(%rbx, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	8(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	8(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	8(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	8(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	8(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	8(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	8(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	8(%rbx, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	8(%rbx, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	8(%rbx, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	16(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	16(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	16(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	16(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	16(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	16(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	16(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	16(%rbx, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	16(%rbx, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	16(%rbx, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	24(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	24(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	24(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	24(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	24(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	24(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	24(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	24(%rbx, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	24(%rbx, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	24(%rbx, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

	subl	$ 4, %r10d
	addq	$ 32, %r12
	addq	$ 32, %rax
	addq	$ 32, %rbx
	addq	$ 128, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	0(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	0(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	0(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	0(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	0(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	0(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	0(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	0(%rbx, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	0(%rbx, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	0(%rbx, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	8(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	8(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	8(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	8(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	8(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	8(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	8(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	8(%rbx, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	8(%rbx, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	8(%rbx, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	16(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	16(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	16(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	16(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	16(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	16(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	16(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	16(%rbx, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	16(%rbx, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	16(%rbx, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	24(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	24(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	24(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	24(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	24(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	24(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	24(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	24(%rbx, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	24(%rbx, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	24(%rbx, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

	subl	$ 4, %r10d
	addq	$ 32, %r12
	addq	$ 32, %rax
	addq	$ 32, %rbx
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
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	0(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	0(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	0(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	0(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	0(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	0(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	0(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	0(%rbx, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	0(%rbx, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	0(%rbx, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

	subl	$ 1, %r10d
	addq	$ 8, %r12
	addq	$ 8, %rax
	addq	$ 8, %rbx
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_4x12_lib4c)
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
	.macro INNER_KERNEL_DGEMM_NN_4X11_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_4x11_lib4c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	leaq	0(%r13, %r13, 2), %r15
	leaq	0(%r12, %r13, 4), %rax
	leaq	0(%r12, %r13, 8), %rbx

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
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	0(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	0(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	0(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	0(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	0(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	0(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	0(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	0(%rbx, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	0(%rbx, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	8(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	8(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	8(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	8(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	8(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	8(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	8(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	8(%rbx, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	8(%rbx, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	16(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	16(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	16(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	16(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	16(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	16(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	16(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	16(%rbx, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	16(%rbx, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	24(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	24(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	24(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	24(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	24(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	24(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	24(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	24(%rbx, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	24(%rbx, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10

	subl	$ 4, %r10d
	addq	$ 32, %r12
	addq	$ 32, %rax
	addq	$ 32, %rbx
	addq	$ 128, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	0(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	0(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	0(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	0(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	0(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	0(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	0(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	0(%rbx, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	0(%rbx, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	8(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	8(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	8(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	8(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	8(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	8(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	8(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	8(%rbx, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	8(%rbx, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	16(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	16(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	16(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	16(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	16(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	16(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	16(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	16(%rbx, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	16(%rbx, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	24(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	24(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	24(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	24(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	24(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	24(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	24(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	24(%rbx, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	24(%rbx, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10

	subl	$ 4, %r10d
	addq	$ 32, %r12
	addq	$ 32, %rax
	addq	$ 32, %rbx
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
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	0(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	0(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	0(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	0(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	0(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	0(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	0(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	0(%rbx, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	0(%rbx, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10

	subl	$ 1, %r10d
	addq	$ 8, %r12
	addq	$ 8, %rax
	addq	$ 8, %rbx
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_4x11_lib4c)
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
	.macro INNER_KERNEL_DGEMM_NN_4X10_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_4x10_lib4c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	leaq	0(%r13, %r13, 2), %r15
	leaq	0(%r12, %r13, 4), %rax
	leaq	0(%r12, %r13, 8), %rbx

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
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	0(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	0(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	0(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	0(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	0(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	0(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	0(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	0(%rbx, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	8(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	8(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	8(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	8(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	8(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	8(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	8(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	8(%rbx, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	16(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	16(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	16(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	16(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	16(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	16(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	16(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	16(%rbx, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	24(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	24(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	24(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	24(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	24(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	24(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	24(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	24(%rbx, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9

	subl	$ 4, %r10d
	addq	$ 32, %r12
	addq	$ 32, %rax
	addq	$ 32, %rbx
	addq	$ 128, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	0(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	0(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	0(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	0(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	0(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	0(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	0(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	0(%rbx, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	8(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	8(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	8(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	8(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	8(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	8(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	8(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	8(%rbx, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	16(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	16(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	16(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	16(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	16(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	16(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	16(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	16(%rbx, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	24(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	24(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	24(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	24(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	24(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	24(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	24(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	24(%rbx, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9

	subl	$ 4, %r10d
	addq	$ 32, %r12
	addq	$ 32, %rax
	addq	$ 32, %rbx
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
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	0(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	0(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	0(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	0(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	0(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	0(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	0(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	0(%rbx, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9

	subl	$ 1, %r10d
	addq	$ 8, %r12
	addq	$ 8, %rax
	addq	$ 8, %rbx
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_4x10_lib4c)
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
	.macro INNER_KERNEL_DGEMM_NN_4X9_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_4x9_lib4c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	leaq	0(%r13, %r13, 2), %r15
	leaq	0(%r12, %r13, 4), %rax
	leaq	0(%r12, %r13, 8), %rbx

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
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	0(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	0(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	0(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	0(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	0(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	0(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	0(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	8(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	8(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	8(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	8(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	8(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	8(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	8(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	16(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	16(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	16(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	16(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	16(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	16(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	16(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	24(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	24(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	24(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	24(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	24(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	24(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	24(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8

	subl	$ 4, %r10d
	addq	$ 32, %r12
	addq	$ 32, %rax
	addq	$ 32, %rbx
	addq	$ 128, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	0(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	0(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	0(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	0(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	0(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	0(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	0(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	8(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	8(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	8(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	8(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	8(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	8(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	8(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	16(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	16(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	16(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	16(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	16(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	16(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	16(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	24(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	24(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	24(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	24(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	24(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	24(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	24(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8

	subl	$ 4, %r10d
	addq	$ 32, %r12
	addq	$ 32, %rax
	addq	$ 32, %rbx
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
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	0(%r12, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	0(%r12, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	0(%r12, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	0(%rax, %r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	0(%rax, %r13, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	0(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	0(%rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8

	subl	$ 1, %r10d
	addq	$ 8, %r12
	addq	$ 8, %rax
	addq	$ 8, %rbx
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_4x9_lib4c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- lda*sizeof(double)
// r13   <- B
// r14   <- ldb*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_12X4_LIBC4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_12x4_libc4)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	leaq	0(%r12, %r12, 2), %r15

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	leaq	0(%r11, %r12, 4), %rax

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
	vmovupd			64(%r11), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rax)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rax)
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovupd			0(%r11, %r12), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12), %ymm15 // A
	vbroadcastsd	32(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	40(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax, %r12)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	48(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rax, %r12)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	56(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rax, %r12)
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovupd			0(%r11, %r12, 2), %ymm13 // A
	vmovupd			32(%r11, %r12, 2), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	64(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	72(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax, %r12, 2)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	80(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rax, %r12, 2)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	88(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rax, %r12, 2)
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovupd			0(%r11, %r15), %ymm13 // A
	vmovupd			32(%r11, %r15), %ymm14 // A
	vmovupd			64(%r11, %r15), %ymm15 // A
	vbroadcastsd	96(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	104(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax, %r15)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	112(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rax, %r15)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	120(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rax, %r15)
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	addq	$ 128, %r13
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl	$ 4, %r10d
	leaq	0(%r11, %r12, 4), %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
	vmovupd			64(%r11), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovupd			0(%r11, %r12), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12), %ymm15 // A
	vbroadcastsd	32(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	40(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	48(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	56(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovupd			0(%r11, %r12, 2), %ymm13 // A
	vmovupd			32(%r11, %r12, 2), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	64(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	72(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	80(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	88(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovupd			0(%r11, %r15), %ymm13 // A
	vmovupd			32(%r11, %r15), %ymm14 // A
	vmovupd			64(%r11, %r15), %ymm15 // A
	vbroadcastsd	96(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	104(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	112(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	120(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	addq	$ 128, %r13
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl	$ 4, %r10d
	leaq	0(%r11, %r12, 4), %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
	vmovupd			64(%r11), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	$ 32, %r13

	subl	$ 1, %r10d
	addq	%r12, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_12x4_libc4)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- lda*sizeof(double)
// r13   <- B
// r14   <- ldb*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_12X3_LIBC4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_12x3_libc4)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	leaq	0(%r12, %r12, 2), %r15

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	leaq	0(%r11, %r12, 4), %rax

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
	vmovupd			64(%r11), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rax)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
//	vbroadcastsd	24(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rax)
//	vfmadd231pd		%ymm14, %ymm12, %ymm7
//	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovupd			0(%r11, %r12), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12), %ymm15 // A
	vbroadcastsd	32(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	40(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax, %r12)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	48(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rax, %r12)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
//	vbroadcastsd	56(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rax, %r12)
//	vfmadd231pd		%ymm14, %ymm12, %ymm7
//	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovupd			0(%r11, %r12, 2), %ymm13 // A
	vmovupd			32(%r11, %r12, 2), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	64(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	72(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax, %r12, 2)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	80(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rax, %r12, 2)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
//	vbroadcastsd	88(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rax, %r12, 2)
//	vfmadd231pd		%ymm14, %ymm12, %ymm7
//	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovupd			0(%r11, %r15), %ymm13 // A
	vmovupd			32(%r11, %r15), %ymm14 // A
	vmovupd			64(%r11, %r15), %ymm15 // A
	vbroadcastsd	96(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	104(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax, %r15)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	112(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rax, %r15)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
//	vbroadcastsd	120(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rax, %r15)
//	vfmadd231pd		%ymm14, %ymm12, %ymm7
	addq	$ 128, %r13
//	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl	$ 4, %r10d
	leaq	0(%r11, %r12, 4), %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
	vmovupd			64(%r11), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
//	vbroadcastsd	24(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm3
//	vfmadd231pd		%ymm14, %ymm12, %ymm7
//	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovupd			0(%r11, %r12), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12), %ymm15 // A
	vbroadcastsd	32(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	40(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	48(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
//	vbroadcastsd	56(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm3
//	vfmadd231pd		%ymm14, %ymm12, %ymm7
//	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovupd			0(%r11, %r12, 2), %ymm13 // A
	vmovupd			32(%r11, %r12, 2), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	64(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	72(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	80(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
//	vbroadcastsd	88(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm3
//	vfmadd231pd		%ymm14, %ymm12, %ymm7
//	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovupd			0(%r11, %r15), %ymm13 // A
	vmovupd			32(%r11, %r15), %ymm14 // A
	vmovupd			64(%r11, %r15), %ymm15 // A
	vbroadcastsd	96(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	104(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	112(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
//	vbroadcastsd	120(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm3
//	vfmadd231pd		%ymm14, %ymm12, %ymm7
	addq	$ 128, %r13
//	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl	$ 4, %r10d
	leaq	0(%r11, %r12, 4), %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
	vmovupd			64(%r11), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
//	vbroadcastsd	24(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm3
//	vfmadd231pd		%ymm14, %ymm12, %ymm7
//	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	$ 32, %r13

	subl	$ 1, %r10d
	addq	%r12, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_12x3_libc4)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- lda*sizeof(double)
// r13   <- B
// r14   <- ldb*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_12X2_LIBC4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_12x2_libc4)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	leaq	0(%r12, %r12, 2), %r15

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	leaq	0(%r11, %r12, 4), %rax

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
	vmovupd			64(%r11), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
//	vbroadcastsd	16(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rax)
//	vfmadd231pd		%ymm14, %ymm12, %ymm6
//	vfmadd231pd		%ymm15, %ymm12, %ymm10
//	vbroadcastsd	24(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rax)
//	vfmadd231pd		%ymm14, %ymm12, %ymm7
//	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovupd			0(%r11, %r12), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12), %ymm15 // A
	vbroadcastsd	32(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	40(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax, %r12)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
//	vbroadcastsd	48(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rax, %r12)
//	vfmadd231pd		%ymm14, %ymm12, %ymm6
//	vfmadd231pd		%ymm15, %ymm12, %ymm10
//	vbroadcastsd	56(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rax, %r12)
//	vfmadd231pd		%ymm14, %ymm12, %ymm7
//	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovupd			0(%r11, %r12, 2), %ymm13 // A
	vmovupd			32(%r11, %r12, 2), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	64(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	72(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax, %r12, 2)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
//	vbroadcastsd	80(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rax, %r12, 2)
//	vfmadd231pd		%ymm14, %ymm12, %ymm6
//	vfmadd231pd		%ymm15, %ymm12, %ymm10
//	vbroadcastsd	88(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rax, %r12, 2)
//	vfmadd231pd		%ymm14, %ymm12, %ymm7
//	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovupd			0(%r11, %r15), %ymm13 // A
	vmovupd			32(%r11, %r15), %ymm14 // A
	vmovupd			64(%r11, %r15), %ymm15 // A
	vbroadcastsd	96(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	104(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax, %r15)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
//	vbroadcastsd	112(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rax, %r15)
//	vfmadd231pd		%ymm14, %ymm12, %ymm6
//	vfmadd231pd		%ymm15, %ymm12, %ymm10
//	vbroadcastsd	120(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rax, %r15)
//	vfmadd231pd		%ymm14, %ymm12, %ymm7
	addq	$ 128, %r13
//	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl	$ 4, %r10d
	leaq	0(%r11, %r12, 4), %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
	vmovupd			64(%r11), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
//	vbroadcastsd	16(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	vfmadd231pd		%ymm14, %ymm12, %ymm6
//	vfmadd231pd		%ymm15, %ymm12, %ymm10
//	vbroadcastsd	24(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm3
//	vfmadd231pd		%ymm14, %ymm12, %ymm7
//	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovupd			0(%r11, %r12), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12), %ymm15 // A
	vbroadcastsd	32(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	40(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
//	vbroadcastsd	48(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	vfmadd231pd		%ymm14, %ymm12, %ymm6
//	vfmadd231pd		%ymm15, %ymm12, %ymm10
//	vbroadcastsd	56(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm3
//	vfmadd231pd		%ymm14, %ymm12, %ymm7
//	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovupd			0(%r11, %r12, 2), %ymm13 // A
	vmovupd			32(%r11, %r12, 2), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	64(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	72(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
//	vbroadcastsd	80(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	vfmadd231pd		%ymm14, %ymm12, %ymm6
//	vfmadd231pd		%ymm15, %ymm12, %ymm10
//	vbroadcastsd	88(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm3
//	vfmadd231pd		%ymm14, %ymm12, %ymm7
//	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovupd			0(%r11, %r15), %ymm13 // A
	vmovupd			32(%r11, %r15), %ymm14 // A
	vmovupd			64(%r11, %r15), %ymm15 // A
	vbroadcastsd	96(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	104(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
//	vbroadcastsd	112(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	vfmadd231pd		%ymm14, %ymm12, %ymm6
//	vfmadd231pd		%ymm15, %ymm12, %ymm10
//	vbroadcastsd	120(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm3
//	vfmadd231pd		%ymm14, %ymm12, %ymm7
	addq	$ 128, %r13
//	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl	$ 4, %r10d
	leaq	0(%r11, %r12, 4), %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
	vmovupd			64(%r11), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
//	vbroadcastsd	16(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	vfmadd231pd		%ymm14, %ymm12, %ymm6
//	vfmadd231pd		%ymm15, %ymm12, %ymm10
//	vbroadcastsd	24(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm3
//	vfmadd231pd		%ymm14, %ymm12, %ymm7
//	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	$ 32, %r13

	subl	$ 1, %r10d
	addq	%r12, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_12x2_libc4)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- lda*sizeof(double)
// r13   <- B
// r14   <- ldb*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_12X1_LIBC4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_12x1_libc4)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	leaq	0(%r12, %r12, 2), %r15

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	leaq	0(%r11, %r12, 4), %rax

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
	vmovupd			64(%r11), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
//	vbroadcastsd	8(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax)
//	vfmadd231pd		%ymm14, %ymm12, %ymm5
//	vfmadd231pd		%ymm15, %ymm12, %ymm9
//	vbroadcastsd	16(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rax)
//	vfmadd231pd		%ymm14, %ymm12, %ymm6
//	vfmadd231pd		%ymm15, %ymm12, %ymm10
//	vbroadcastsd	24(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rax)
//	vfmadd231pd		%ymm14, %ymm12, %ymm7
//	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovupd			0(%r11, %r12), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12), %ymm15 // A
	vbroadcastsd	32(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
//	vbroadcastsd	40(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax, %r12)
//	vfmadd231pd		%ymm14, %ymm12, %ymm5
//	vfmadd231pd		%ymm15, %ymm12, %ymm9
//	vbroadcastsd	48(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rax, %r12)
//	vfmadd231pd		%ymm14, %ymm12, %ymm6
//	vfmadd231pd		%ymm15, %ymm12, %ymm10
//	vbroadcastsd	56(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rax, %r12)
//	vfmadd231pd		%ymm14, %ymm12, %ymm7
//	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovupd			0(%r11, %r12, 2), %ymm13 // A
	vmovupd			32(%r11, %r12, 2), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	64(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
//	vbroadcastsd	72(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax, %r12, 2)
//	vfmadd231pd		%ymm14, %ymm12, %ymm5
//	vfmadd231pd		%ymm15, %ymm12, %ymm9
//	vbroadcastsd	80(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rax, %r12, 2)
//	vfmadd231pd		%ymm14, %ymm12, %ymm6
//	vfmadd231pd		%ymm15, %ymm12, %ymm10
//	vbroadcastsd	88(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rax, %r12, 2)
//	vfmadd231pd		%ymm14, %ymm12, %ymm7
//	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovupd			0(%r11, %r15), %ymm13 // A
	vmovupd			32(%r11, %r15), %ymm14 // A
	vmovupd			64(%r11, %r15), %ymm15 // A
	vbroadcastsd	96(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
//	vbroadcastsd	104(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax, %r15)
//	vfmadd231pd		%ymm14, %ymm12, %ymm5
//	vfmadd231pd		%ymm15, %ymm12, %ymm9
//	vbroadcastsd	112(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rax, %r15)
//	vfmadd231pd		%ymm14, %ymm12, %ymm6
//	vfmadd231pd		%ymm15, %ymm12, %ymm10
//	vbroadcastsd	120(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rax, %r15)
//	vfmadd231pd		%ymm14, %ymm12, %ymm7
	addq	$ 128, %r13
//	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl	$ 4, %r10d
	leaq	0(%r11, %r12, 4), %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
	vmovupd			64(%r11), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
//	vbroadcastsd	8(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm1
//	vfmadd231pd		%ymm14, %ymm12, %ymm5
//	vfmadd231pd		%ymm15, %ymm12, %ymm9
//	vbroadcastsd	16(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	vfmadd231pd		%ymm14, %ymm12, %ymm6
//	vfmadd231pd		%ymm15, %ymm12, %ymm10
//	vbroadcastsd	24(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm3
//	vfmadd231pd		%ymm14, %ymm12, %ymm7
//	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovupd			0(%r11, %r12), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12), %ymm15 // A
	vbroadcastsd	32(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
//	vbroadcastsd	40(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm1
//	vfmadd231pd		%ymm14, %ymm12, %ymm5
//	vfmadd231pd		%ymm15, %ymm12, %ymm9
//	vbroadcastsd	48(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	vfmadd231pd		%ymm14, %ymm12, %ymm6
//	vfmadd231pd		%ymm15, %ymm12, %ymm10
//	vbroadcastsd	56(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm3
//	vfmadd231pd		%ymm14, %ymm12, %ymm7
//	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovupd			0(%r11, %r12, 2), %ymm13 // A
	vmovupd			32(%r11, %r12, 2), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	64(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
//	vbroadcastsd	72(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm1
//	vfmadd231pd		%ymm14, %ymm12, %ymm5
//	vfmadd231pd		%ymm15, %ymm12, %ymm9
//	vbroadcastsd	80(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	vfmadd231pd		%ymm14, %ymm12, %ymm6
//	vfmadd231pd		%ymm15, %ymm12, %ymm10
//	vbroadcastsd	88(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm3
//	vfmadd231pd		%ymm14, %ymm12, %ymm7
//	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovupd			0(%r11, %r15), %ymm13 // A
	vmovupd			32(%r11, %r15), %ymm14 // A
	vmovupd			64(%r11, %r15), %ymm15 // A
	vbroadcastsd	96(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
//	vbroadcastsd	104(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm1
//	vfmadd231pd		%ymm14, %ymm12, %ymm5
//	vfmadd231pd		%ymm15, %ymm12, %ymm9
//	vbroadcastsd	112(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	vfmadd231pd		%ymm14, %ymm12, %ymm6
//	vfmadd231pd		%ymm15, %ymm12, %ymm10
//	vbroadcastsd	120(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm3
//	vfmadd231pd		%ymm14, %ymm12, %ymm7
	addq	$ 128, %r13
//	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl	$ 4, %r10d
	leaq	0(%r11, %r12, 4), %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
	vmovupd			64(%r11), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
//	vbroadcastsd	8(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm1
//	vfmadd231pd		%ymm14, %ymm12, %ymm5
//	vfmadd231pd		%ymm15, %ymm12, %ymm9
//	vbroadcastsd	16(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	vfmadd231pd		%ymm14, %ymm12, %ymm6
//	vfmadd231pd		%ymm15, %ymm12, %ymm10
//	vbroadcastsd	24(%r13), %ymm12 // B
//	vfmadd231pd		%ymm13, %ymm12, %ymm3
//	vfmadd231pd		%ymm14, %ymm12, %ymm7
//	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	$ 32, %r13

	subl	$ 1, %r10d
	addq	%r12, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_12x1_libc4)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- lda*sizeof(double)
// r13   <- B
// r14   <- ldb*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_12X4_LIBCC
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_12x4_libcc)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	leaq	0(%r12, %r12, 2), %r15

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	leaq	0(%r11, %r12, 4), %rax

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
	vmovupd			64(%r11), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rax)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rax)
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 1
	vmovupd			0(%r11, %r12), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax, %r12)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rax, %r12)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rax, %r12)
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 2
	vmovupd			0(%r11, %r12, 2), %ymm13 // A
	vmovupd			32(%r11, %r12, 2), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax, %r12, 2)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rax, %r12, 2)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rax, %r12, 2)
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 3
	vmovupd			0(%r11, %r15), %ymm13 // A
	vmovupd			32(%r11, %r15), %ymm14 // A
	vmovupd			64(%r11, %r15), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax, %r15)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rax, %r15)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rax, %r15)
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	subl	$ 4, %r10d
	leaq	0(%r11, %r12, 4), %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
	vmovupd			64(%r11), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
//	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 1
	vmovupd			0(%r11, %r12), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
//	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 2
	vmovupd			0(%r11, %r12, 2), %ymm13 // A
	vmovupd			32(%r11, %r12, 2), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
//	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 3
	vmovupd			0(%r11, %r15), %ymm13 // A
	vmovupd			32(%r11, %r15), %ymm14 // A
	vmovupd			64(%r11, %r15), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
//	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	subl	$ 4, %r10d
	leaq	0(%r11, %r12, 4), %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
	vmovupd			64(%r11), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
//	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	subl	$ 1, %r10d
	addq	%r12, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_12x4_libcc)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- lda*sizeof(double)
// r13   <- B
// r14   <- ldb*sizeof(double)
// r15   <- km
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_12X4_VS_LIBCC
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_12x4_vs_libcc)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	vcvtsi2sd	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC04(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC04(%rip), %ymm13
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm13, %ymm15

	vmovupd		%ymm15, -32(%rsp)

	leaq	0(%r12, %r12, 2), %rbx

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	leaq	0(%r11, %r12, 4), %rax

	// unroll 0
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
//	vmovupd			64(%r11), %ymm15 // A
	vmaskmovpd		64(%r11), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rax)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rax)
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 1
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
//	vmovupd			64(%r11, %r12), %ymm15 // A
	vmaskmovpd		64(%r11, %r12), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax, %r12)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rax, %r12)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rax, %r12)
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 2
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12, 2), %ymm13 // A
	vmovupd			32(%r11, %r12, 2), %ymm14 // A
//	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vmaskmovpd		64(%r11, %r12, 2), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax, %r12, 2)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rax, %r12, 2)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rax, %r12, 2)
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 3
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %rbx), %ymm13 // A
	vmovupd			32(%r11, %rbx), %ymm14 // A
//	vmovupd			64(%r11, %rbx), %ymm15 // A
	vmaskmovpd		64(%r11, %rbx), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax, %rbx)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rax, %rbx)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rax, %rbx)
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	subl	$ 4, %r10d
	leaq	0(%r11, %r12, 4), %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
//	vmovupd			64(%r11), %ymm15 // A
	vmaskmovpd		64(%r11), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
//	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 1
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
//	vmovupd			64(%r11, %r12), %ymm15 // A
	vmaskmovpd		64(%r11, %r12), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
//	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 2
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12, 2), %ymm13 // A
	vmovupd			32(%r11, %r12, 2), %ymm14 // A
//	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vmaskmovpd		64(%r11, %r12, 2), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
//	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 3
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %rbx), %ymm13 // A
	vmovupd			32(%r11, %rbx), %ymm14 // A
//	vmovupd			64(%r11, %rbx), %ymm15 // A
	vmaskmovpd		64(%r11, %rbx), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
//	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	subl	$ 4, %r10d
	leaq	0(%r11, %r12, 4), %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
//	vmovupd			64(%r11), %ymm15 // A
	vmaskmovpd		64(%r11), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
//	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	subl	$ 1, %r10d
	addq	%r12, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_12x4_vs_libcc)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- lda*sizeof(double)
// r13   <- B
// r14   <- ldb*sizeof(double)
// r15   <- km
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_12X3_VS_LIBCC
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_12x3_vs_libcc)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	vcvtsi2sd	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC04(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC04(%rip), %ymm13
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm13, %ymm15

	vmovupd		%ymm15, -32(%rsp)

	leaq	0(%r12, %r12, 2), %rbx

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	leaq	0(%r11, %r12, 4), %rax

	// unroll 0
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
//	vmovupd			64(%r11), %ymm15 // A
	vmaskmovpd		64(%r11), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	prefetcht0		64(%rax)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	addq	%r14, %r13

	// unroll 1
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
//	vmovupd			64(%r11, %r12), %ymm15 // A
	vmaskmovpd		64(%r11, %r12), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax, %r12)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	prefetcht0		64(%rax, %r12)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	addq	%r14, %r13

	// unroll 2
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12, 2), %ymm13 // A
	vmovupd			32(%r11, %r12, 2), %ymm14 // A
//	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vmaskmovpd		64(%r11, %r12, 2), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax, %r12, 2)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	prefetcht0		64(%rax, %r12, 2)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	addq	%r14, %r13

	// unroll 3
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %rbx), %ymm13 // A
	vmovupd			32(%r11, %rbx), %ymm14 // A
//	vmovupd			64(%r11, %rbx), %ymm15 // A
	vmaskmovpd		64(%r11, %rbx), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax, %rbx)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	prefetcht0		64(%rax, %rbx)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	addq	%r14, %r13

	subl	$ 4, %r10d
	leaq	0(%r11, %r12, 4), %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
//	vmovupd			64(%r11), %ymm15 // A
	vmaskmovpd		64(%r11), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
//	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	addq	%r14, %r13

	// unroll 1
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
//	vmovupd			64(%r11, %r12), %ymm15 // A
	vmaskmovpd		64(%r11, %r12), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
//	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	addq	%r14, %r13

	// unroll 2
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12, 2), %ymm13 // A
	vmovupd			32(%r11, %r12, 2), %ymm14 // A
//	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vmaskmovpd		64(%r11, %r12, 2), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
//	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	addq	%r14, %r13

	// unroll 3
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %rbx), %ymm13 // A
	vmovupd			32(%r11, %rbx), %ymm14 // A
//	vmovupd			64(%r11, %rbx), %ymm15 // A
	vmaskmovpd		64(%r11, %rbx), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
//	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	addq	%r14, %r13

	subl	$ 4, %r10d
	leaq	0(%r11, %r12, 4), %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
//	vmovupd			64(%r11), %ymm15 // A
	vmaskmovpd		64(%r11), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
//	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	addq	%r14, %r13

	subl	$ 1, %r10d
	addq	%r12, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_12x3_vs_libcc)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- lda*sizeof(double)
// r13   <- B
// r14   <- ldb*sizeof(double)
// r15   <- km
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_12X2_VS_LIBCC
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_12x2_vs_libcc)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	vcvtsi2sd	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC04(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC04(%rip), %ymm13
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm13, %ymm15

	vmovupd		%ymm15, -32(%rsp)

	leaq	0(%r12, %r12, 2), %rbx

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	leaq	0(%r11, %r12, 4), %rax

	// unroll 0
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
//	vmovupd			64(%r11), %ymm15 // A
	vmaskmovpd		64(%r11), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	prefetcht0		64(%rax)
	addq	%r14, %r13

	// unroll 1
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
//	vmovupd			64(%r11, %r12), %ymm15 // A
	vmaskmovpd		64(%r11, %r12), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax, %r12)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	prefetcht0		64(%rax, %r12)
	addq	%r14, %r13

	// unroll 2
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12, 2), %ymm13 // A
	vmovupd			32(%r11, %r12, 2), %ymm14 // A
//	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vmaskmovpd		64(%r11, %r12, 2), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax, %r12, 2)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	prefetcht0		64(%rax, %r12, 2)
	addq	%r14, %r13

	// unroll 3
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %rbx), %ymm13 // A
	vmovupd			32(%r11, %rbx), %ymm14 // A
//	vmovupd			64(%r11, %rbx), %ymm15 // A
	vmaskmovpd		64(%r11, %rbx), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rax, %rbx)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	prefetcht0		64(%rax, %rbx)
	addq	%r14, %r13

	subl	$ 4, %r10d
	leaq	0(%r11, %r12, 4), %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
//	vmovupd			64(%r11), %ymm15 // A
	vmaskmovpd		64(%r11), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
//	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	addq	%r14, %r13

	// unroll 1
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
//	vmovupd			64(%r11, %r12), %ymm15 // A
	vmaskmovpd		64(%r11, %r12), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
//	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	addq	%r14, %r13

	// unroll 2
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12, 2), %ymm13 // A
	vmovupd			32(%r11, %r12, 2), %ymm14 // A
//	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vmaskmovpd		64(%r11, %r12, 2), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
//	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	addq	%r14, %r13

	// unroll 3
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %rbx), %ymm13 // A
	vmovupd			32(%r11, %rbx), %ymm14 // A
//	vmovupd			64(%r11, %rbx), %ymm15 // A
	vmaskmovpd		64(%r11, %rbx), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
//	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	addq	%r14, %r13

	subl	$ 4, %r10d
	leaq	0(%r11, %r12, 4), %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
//	vmovupd			64(%r11), %ymm15 // A
	vmaskmovpd		64(%r11), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
//	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	addq	%r14, %r13

	subl	$ 1, %r10d
	addq	%r12, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_12x2_vs_libcc)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- lda*sizeof(double)
// r13   <- B
// r14   <- ldb*sizeof(double)
// r15   <- km
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_12X1_VS_LIBCC
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_12x1_vs_libcc)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	vcvtsi2sd	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC04(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC04(%rip), %ymm13
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm13, %ymm15

	vmovupd		%ymm15, -32(%rsp)

	leaq	0(%r12, %r12, 2), %rbx

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	leaq	0(%r11, %r12, 4), %rax

	// unroll 0
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
//	vmovupd			64(%r11), %ymm15 // A
	vmaskmovpd		64(%r11), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	prefetcht0		0(%rax)
	prefetcht0		64(%rax)
	addq	%r14, %r13

	// unroll 1
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
//	vmovupd			64(%r11, %r12), %ymm15 // A
	vmaskmovpd		64(%r11, %r12), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	prefetcht0		0(%rax, %r12)
	prefetcht0		64(%rax, %r12)
	addq	%r14, %r13

	// unroll 2
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12, 2), %ymm13 // A
	vmovupd			32(%r11, %r12, 2), %ymm14 // A
//	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vmaskmovpd		64(%r11, %r12, 2), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	prefetcht0		0(%rax, %r12, 2)
	prefetcht0		64(%rax, %r12, 2)
	addq	%r14, %r13

	// unroll 3
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %rbx), %ymm13 // A
	vmovupd			32(%r11, %rbx), %ymm14 // A
//	vmovupd			64(%r11, %rbx), %ymm15 // A
	vmaskmovpd		64(%r11, %rbx), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	prefetcht0		0(%rax, %rbx)
	prefetcht0		64(%rax, %rbx)
	addq	%r14, %r13

	subl	$ 4, %r10d
	leaq	0(%r11, %r12, 4), %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
//	vmovupd			64(%r11), %ymm15 // A
	vmaskmovpd		64(%r11), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
//	prefetcht0		0(%r13, %r14, 8)
	addq	%r14, %r13

	// unroll 1
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
//	vmovupd			64(%r11, %r12), %ymm15 // A
	vmaskmovpd		64(%r11, %r12), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
//	prefetcht0		0(%r13, %r14, 8)
	addq	%r14, %r13

	// unroll 2
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12, 2), %ymm13 // A
	vmovupd			32(%r11, %r12, 2), %ymm14 // A
//	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vmaskmovpd		64(%r11, %r12, 2), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
//	prefetcht0		0(%r13, %r14, 8)
	addq	%r14, %r13

	// unroll 3
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %rbx), %ymm13 // A
	vmovupd			32(%r11, %rbx), %ymm14 // A
//	vmovupd			64(%r11, %rbx), %ymm15 // A
	vmaskmovpd		64(%r11, %rbx), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
//	prefetcht0		0(%r13, %r14, 8)
	addq	%r14, %r13

	subl	$ 4, %r10d
	leaq	0(%r11, %r12, 4), %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
//	vmovupd			64(%r11), %ymm15 // A
	vmaskmovpd		64(%r11), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
//	prefetcht0		0(%r13, %r14, 8)
	addq	%r14, %r13

	subl	$ 1, %r10d
	addq	%r12, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_12x1_vs_libcc)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- lda*sizeof(double)
// r13   <- B
// r14   <- ldb*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_12X4_LIBCC
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_12x4_libcc)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	leaq	0(%r12, %r12, 2), %rax
	leaq	0(%r14, %r14, 2), %r15

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	0(%r12, %r13, 2) // software prefetch
//	prefetcht0	64(%r12, %r13, 2) // software prefetch

	leaq	0(%r11, %r12, 4), %rbx

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
	vmovupd			64(%r11), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rbx)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rbx)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	0(%r13, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rbx)
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovupd			0(%r11, %r12), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12), %ymm15 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rbx, %r12)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	8(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rbx, %r12)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	8(%r13, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rbx, %r12)
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovupd			0(%r11, %r12, 2), %ymm13 // A
	vmovupd			32(%r11, %r12, 2), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	16(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rbx, %r12, 2)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rbx, %r12, 2)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	16(%r13, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rbx, %r12, 2)
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovupd			0(%r11, %rax), %ymm13 // A
	vmovupd			32(%r11, %rax), %ymm14 // A
	vmovupd			64(%r11, %rax), %ymm15 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	24(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rbx, %rax)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	24(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rbx, %rax)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rbx, %rax)
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl	$ 4, %r10d
	addq	$ 32, %r13
	leaq	0(%r11, %r12, 4), %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
	vmovupd			64(%r11), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	0(%r13, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovupd			0(%r11, %r12), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12), %ymm15 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	8(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	8(%r13, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovupd			0(%r11, %r12, 2), %ymm13 // A
	vmovupd			32(%r11, %r12, 2), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	16(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	16(%r13, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovupd			0(%r11, %rax), %ymm13 // A
	vmovupd			32(%r11, %rax), %ymm14 // A
	vmovupd			64(%r11, %rax), %ymm15 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	24(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	24(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl	$ 4, %r10d
	addq	$ 32, %r13
	leaq	0(%r11, %r12, 4), %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
	vmovupd			64(%r11), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	0(%r13, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl	$ 1, %r10d
	addq	$ 8, %r13
	addq	%r12, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_12x4_libcc)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- lda*sizeof(double)
// r13   <- B
// r14   <- ldb*sizeof(double)
// r15   <- km
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_12X4_VS_LIBCC
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_12x4_vs_libcc)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	vcvtsi2sd	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC04(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC04(%rip), %ymm13
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm13, %ymm15

	vmovupd		%ymm15, -32(%rsp)

	leaq	0(%r12, %r12, 2), %rax
	leaq	0(%r14, %r14, 2), %rbp

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	0(%r12, %r13, 2) // software prefetch
//	prefetcht0	64(%r12, %r13, 2) // software prefetch

	leaq	0(%r11, %r12, 4), %rbx

	// unroll 0
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
//	vmovupd			64(%r11), %ymm15 // A
	vmaskmovpd		64(%r11), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rbx)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rbx)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	0(%r13, %rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rbx)
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
//	vmovupd			64(%r11, %r12), %ymm15 // A
	vmaskmovpd		64(%r11, %r12), %ymm15, %ymm15 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rbx, %r12)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	8(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rbx, %r12)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	8(%r13, %rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rbx, %r12)
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12, 2), %ymm13 // A
	vmovupd			32(%r11, %r12, 2), %ymm14 // A
//	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vmaskmovpd		64(%r11, %r12, 2), %ymm15, %ymm15 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	16(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rbx, %r12, 2)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rbx, %r12, 2)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	16(%r13, %rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rbx, %r12, 2)
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %rax), %ymm13 // A
	vmovupd			32(%r11, %rax), %ymm14 // A
//	vmovupd			64(%r11, %rax), %ymm15 // A
	vmaskmovpd		64(%r11, %rax), %ymm15, %ymm15 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	24(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rbx, %rax)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	24(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rbx, %rax)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13, %rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	prefetcht0		64(%rbx, %rax)
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl	$ 4, %r10d
	addq	$ 32, %r13
	leaq	0(%r11, %r12, 4), %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
//	vmovupd			64(%r11), %ymm15 // A
	vmaskmovpd		64(%r11), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	0(%r13, %rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
//	vmovupd			64(%r11, %r12), %ymm15 // A
	vmaskmovpd		64(%r11, %r12), %ymm15, %ymm15 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	8(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	8(%r13, %rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12, 2), %ymm13 // A
	vmovupd			32(%r11, %r12, 2), %ymm14 // A
//	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vmaskmovpd		64(%r11, %r12, 2), %ymm15, %ymm15 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	16(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	16(%r13, %rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %rax), %ymm13 // A
	vmovupd			32(%r11, %rax), %ymm14 // A
//	vmovupd			64(%r11, %rax), %ymm15 // A
	vmaskmovpd		64(%r11, %rax), %ymm15, %ymm15 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	24(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	24(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13, %rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl	$ 4, %r10d
	addq	$ 32, %r13
	leaq	0(%r11, %r12, 4), %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
//	vmovupd			64(%r11), %ymm15 // A
	vmaskmovpd		64(%r11), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	0(%r13, %rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl	$ 1, %r10d
	addq	$ 8, %r13
	addq	%r12, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_12x4_vs_libcc)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- lda*sizeof(double)
// r13   <- B
// r14   <- ldb*sizeof(double)
// r15   <- km
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_12X3_VS_LIBCC
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_12x3_vs_libcc)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	vcvtsi2sd	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC04(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC04(%rip), %ymm13
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm13, %ymm15

	vmovupd		%ymm15, -32(%rsp)

	leaq	0(%r12, %r12, 2), %rax
//	leaq	0(%r14, %r14, 2), %rbp

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	0(%r12, %r13, 2) // software prefetch
//	prefetcht0	64(%r12, %r13, 2) // software prefetch

	leaq	0(%r11, %r12, 4), %rbx

	// unroll 0
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
//	vmovupd			64(%r11), %ymm15 // A
	vmaskmovpd		64(%r11), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rbx)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	prefetcht0		64(%rbx)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	// unroll 1
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
//	vmovupd			64(%r11, %r12), %ymm15 // A
	vmaskmovpd		64(%r11, %r12), %ymm15, %ymm15 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rbx, %r12)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	8(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	prefetcht0		64(%rbx, %r12)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	// unroll 2
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12, 2), %ymm13 // A
	vmovupd			32(%r11, %r12, 2), %ymm14 // A
//	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vmaskmovpd		64(%r11, %r12, 2), %ymm15, %ymm15 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	16(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rbx, %r12, 2)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	prefetcht0		64(%rbx, %r12, 2)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	// unroll 3
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %rax), %ymm13 // A
	vmovupd			32(%r11, %rax), %ymm14 // A
//	vmovupd			64(%r11, %rax), %ymm15 // A
	vmaskmovpd		64(%r11, %rax), %ymm15, %ymm15 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	24(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rbx, %rax)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	24(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	prefetcht0		64(%rbx, %rax)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	subl	$ 4, %r10d
	addq	$ 32, %r13
	leaq	0(%r11, %r12, 4), %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
//	vmovupd			64(%r11), %ymm15 // A
	vmaskmovpd		64(%r11), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	// unroll 1
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
//	vmovupd			64(%r11, %r12), %ymm15 // A
	vmaskmovpd		64(%r11, %r12), %ymm15, %ymm15 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	8(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	// unroll 2
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12, 2), %ymm13 // A
	vmovupd			32(%r11, %r12, 2), %ymm14 // A
//	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vmaskmovpd		64(%r11, %r12, 2), %ymm15, %ymm15 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	16(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	// unroll 3
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %rax), %ymm13 // A
	vmovupd			32(%r11, %rax), %ymm14 // A
//	vmovupd			64(%r11, %rax), %ymm15 // A
	vmaskmovpd		64(%r11, %rax), %ymm15, %ymm15 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	24(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	24(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	subl	$ 4, %r10d
	addq	$ 32, %r13
	leaq	0(%r11, %r12, 4), %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
//	vmovupd			64(%r11), %ymm15 // A
	vmaskmovpd		64(%r11), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	subl	$ 1, %r10d
	addq	$ 8, %r13
	addq	%r12, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_12x3_vs_libcc)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- lda*sizeof(double)
// r13   <- B
// r14   <- ldb*sizeof(double)
// r15   <- km
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_12X2_VS_LIBCC
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_12x2_vs_libcc)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	vcvtsi2sd	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC04(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC04(%rip), %ymm13
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm13, %ymm15

	vmovupd		%ymm15, -32(%rsp)

	leaq	0(%r12, %r12, 2), %rax
//	leaq	0(%r14, %r14, 2), %rbp

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	0(%r12, %r13, 2) // software prefetch
//	prefetcht0	64(%r12, %r13, 2) // software prefetch

	leaq	0(%r11, %r12, 4), %rbx

	// unroll 0
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
//	vmovupd			64(%r11), %ymm15 // A
	vmaskmovpd		64(%r11), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rbx)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	prefetcht0		64(%rbx)

	// unroll 1
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
//	vmovupd			64(%r11, %r12), %ymm15 // A
	vmaskmovpd		64(%r11, %r12), %ymm15, %ymm15 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rbx, %r12)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	prefetcht0		64(%rbx, %r12)

	// unroll 2
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12, 2), %ymm13 // A
	vmovupd			32(%r11, %r12, 2), %ymm14 // A
//	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vmaskmovpd		64(%r11, %r12, 2), %ymm15, %ymm15 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	16(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rbx, %r12, 2)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	prefetcht0		64(%rbx, %r12, 2)

	// unroll 3
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %rax), %ymm13 // A
	vmovupd			32(%r11, %rax), %ymm14 // A
//	vmovupd			64(%r11, %rax), %ymm15 // A
	vmaskmovpd		64(%r11, %rax), %ymm15, %ymm15 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	24(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rbx, %rax)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	prefetcht0		64(%rbx, %rax)

	subl	$ 4, %r10d
	addq	$ 32, %r13
	leaq	0(%r11, %r12, 4), %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
//	vmovupd			64(%r11), %ymm15 // A
	vmaskmovpd		64(%r11), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	// unroll 1
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
//	vmovupd			64(%r11, %r12), %ymm15 // A
	vmaskmovpd		64(%r11, %r12), %ymm15, %ymm15 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	// unroll 2
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12, 2), %ymm13 // A
	vmovupd			32(%r11, %r12, 2), %ymm14 // A
//	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vmaskmovpd		64(%r11, %r12, 2), %ymm15, %ymm15 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	16(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	// unroll 3
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %rax), %ymm13 // A
	vmovupd			32(%r11, %rax), %ymm14 // A
//	vmovupd			64(%r11, %rax), %ymm15 // A
	vmaskmovpd		64(%r11, %rax), %ymm15, %ymm15 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	24(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	subl	$ 4, %r10d
	addq	$ 32, %r13
	leaq	0(%r11, %r12, 4), %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
//	vmovupd			64(%r11), %ymm15 // A
	vmaskmovpd		64(%r11), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	subl	$ 1, %r10d
	addq	$ 8, %r13
	addq	%r12, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_12x2_vs_libcc)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- lda*sizeof(double)
// r13   <- B
// r14   <- ldb*sizeof(double)
// r15   <- km
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_12X1_VS_LIBCC
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_12x1_vs_libcc)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	vcvtsi2sd	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC04(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC04(%rip), %ymm13
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm13, %ymm15

	vmovupd		%ymm15, -32(%rsp)

	leaq	0(%r12, %r12, 2), %rax
//	leaq	0(%r14, %r14, 2), %rbp

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	0(%r12, %r13, 2) // software prefetch
//	prefetcht0	64(%r12, %r13, 2) // software prefetch

	leaq	0(%r11, %r12, 4), %rbx

	// unroll 0
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
//	vmovupd			64(%r11), %ymm15 // A
	vmaskmovpd		64(%r11), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	prefetcht0		0(%rbx)
	prefetcht0		64(%rbx)

	// unroll 1
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
//	vmovupd			64(%r11, %r12), %ymm15 // A
	vmaskmovpd		64(%r11, %r12), %ymm15, %ymm15 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	prefetcht0		0(%rbx, %r12)
	prefetcht0		64(%rbx, %r12)

	// unroll 2
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12, 2), %ymm13 // A
	vmovupd			32(%r11, %r12, 2), %ymm14 // A
//	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vmaskmovpd		64(%r11, %r12, 2), %ymm15, %ymm15 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	prefetcht0		0(%rbx, %r12, 2)
	prefetcht0		64(%rbx, %r12, 2)

	// unroll 3
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %rax), %ymm13 // A
	vmovupd			32(%r11, %rax), %ymm14 // A
//	vmovupd			64(%r11, %rax), %ymm15 // A
	vmaskmovpd		64(%r11, %rax), %ymm15, %ymm15 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	prefetcht0		0(%rbx, %rax)
	prefetcht0		64(%rbx, %rax)

	subl	$ 4, %r10d
	addq	$ 32, %r13
	leaq	0(%r11, %r12, 4), %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
//	vmovupd			64(%r11), %ymm15 // A
	vmaskmovpd		64(%r11), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	// unroll 1
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
//	vmovupd			64(%r11, %r12), %ymm15 // A
	vmaskmovpd		64(%r11, %r12), %ymm15, %ymm15 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	// unroll 2
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %r12, 2), %ymm13 // A
	vmovupd			32(%r11, %r12, 2), %ymm14 // A
//	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vmaskmovpd		64(%r11, %r12, 2), %ymm15, %ymm15 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	// unroll 3
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11, %rax), %ymm13 // A
	vmovupd			32(%r11, %rax), %ymm14 // A
//	vmovupd			64(%r11, %rax), %ymm15 // A
	vmaskmovpd		64(%r11, %rax), %ymm15, %ymm15 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	subl	$ 4, %r10d
	addq	$ 32, %r13
	leaq	0(%r11, %r12, 4), %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovupd			-32(%rsp), %ymm15 // mask
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			32(%r11), %ymm14 // A
//	vmovupd			64(%r11), %ymm15 // A
	vmaskmovpd		64(%r11), %ymm15, %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	subl	$ 1, %r10d
	addq	$ 8, %r13
	addq	%r12, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_12x1_vs_libcc)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- lda*sizeof(double)
// r13   <- B
// r14   <- ldb*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_4X12_LIBCC
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_4x12_libcc)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	leaq	0(%r14, %r14, 2), %r15
	leaq	0(%r13, %r14, 4), %rbp
	leaq	0(%r13, %r14, 8), %rax

	// preload

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	0(%r12, %r13, 2) // software prefetch
//	prefetcht0	64(%r12, %r13, 2) // software prefetch

	leaq	0(%r11, %r12, 4), %rbx

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rbx)
	vbroadcastsd	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rbx)
	vbroadcastsd	0(%r13, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	0(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	0(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	0(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	0(%rbp, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	0(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	0(%rax, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	0(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

	// unroll 1
	vmovupd			0(%r11, %r12), %ymm13 // A
	leaq	0(%r11, %r12, 2), %r11
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rbx, %r12)
	vbroadcastsd	8(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rbx, %r12)
	vbroadcastsd	8(%r13, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	8(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	8(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	8(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	8(%rbp, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	8(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	8(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	8(%rax, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	8(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

	leaq	0(%rbx, %r12, 2), %rbx

	// unroll 2
	vmovupd			0(%r11), %ymm13 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	16(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rbx)
	vbroadcastsd	16(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rbx)
	vbroadcastsd	16(%r13, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	16(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	16(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	16(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	16(%rbp, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	16(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	16(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	16(%rax, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	16(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

	// unroll 3
	vmovupd			0(%r11, %r12), %ymm13 // A
	leaq	0(%r11, %r12, 2), %r11
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	24(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%rbx, %r12)
	vbroadcastsd	24(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rbx, %r12)
	vbroadcastsd	24(%r13, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	24(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	24(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	24(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	24(%rbp, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	24(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	24(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	24(%rax, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	24(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

	subl	$ 4, %r10d
	addq	$ 32, %r13
	addq	$ 32, %rbp
	addq	$ 32, %rax

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	0(%r13, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	0(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	0(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	0(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	0(%rbp, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	0(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	0(%rax, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	0(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

	// unroll 1
	vmovupd			0(%r11, %r12), %ymm13 // A
	leaq	0(%r11, %r12, 2), %r11
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	8(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	8(%r13, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	8(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	8(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	8(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	8(%rbp, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	8(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	8(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	8(%rax, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	8(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

	// unroll 2
	vmovupd			0(%r11), %ymm13 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	16(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	16(%r13, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	16(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	16(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	16(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	16(%rbp, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	16(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	16(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	16(%rax, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	16(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

	// unroll 3
	vmovupd			0(%r11, %r12), %ymm13 // A
	leaq	0(%r11, %r12, 2), %r11
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	24(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	24(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r13, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	24(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	24(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	24(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	24(%rbp, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	24(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	24(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	24(%rax, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	24(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

	subl	$ 4, %r10d
	addq	$ 32, %r13
	addq	$ 32, %rbp
	addq	$ 32, %rax

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	0(%r13, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	0(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	0(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	0(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	0(%rbp, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	0(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	0(%rax, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	0(%rax, %r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

	subl	$ 1, %r10d
	addq	$ 8, %r13
	addq	$ 8, %rbp
	addq	$ 8, %rax
	addq	%r12, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_4x12_libcc)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- lda*sizeof(double)
// r13   <- B
// r14   <- ldb*sizeof(double)
// r15   <- km
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_4X12_VS_LIBCC
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_4x12_vs_libcc)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	vcvtsi2sd	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %ymm13
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm13, %ymm15

	leaq	0(%r14, %r14, 2), %rbx
	leaq	0(%r13, %r14, 4), %rbp
	leaq	0(%r13, %r14, 8), %rax

	// preload

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	0(%r12, %r13, 2) // software prefetch
//	prefetcht0	64(%r12, %r13, 2) // software prefetch

//	leaq	0(%r11, %r12, 4), %rbx

	// unroll 0
//	vmovupd			0(%r11), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r11, %r12, 4)
	vbroadcastsd	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%r11, %r12, 4)
	vbroadcastsd	0(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	addq	%r12, %r11
	vbroadcastsd	0(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	0(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	0(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	0(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	0(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	0(%rax, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	0(%rax, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

	// unroll 1
//	vmovupd			0(%r11), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r11, %r12, 4)
	vbroadcastsd	8(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%r11, %r12, 4)
	vbroadcastsd	8(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	addq	%r12, %r11
	vbroadcastsd	8(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	8(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	8(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	8(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	8(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	8(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	8(%rax, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	8(%rax, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

//	leaq	0(%r11, %r12, 2), %r11
//	leaq	0(%rbx, %r12, 2), %rbx

	// unroll 2
//	vmovupd			0(%r11), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	16(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r11)
	vbroadcastsd	16(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rbx)
	vbroadcastsd	16(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	addq	%r12, %r11
	vbroadcastsd	16(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	16(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	16(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	16(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	16(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	16(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	16(%rax, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	16(%rax, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

	// unroll 3
//	vmovupd			0(%r11), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	24(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r11, %r12, 4)
	vbroadcastsd	24(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%r11, %r12, 4)
	vbroadcastsd	24(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	addq	%r12, %r11
	vbroadcastsd	24(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	24(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	24(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	24(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	24(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	24(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	24(%rax, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	24(%rax, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

//	leaq	0(%r11, %r12, 2), %r11

	subl	$ 4, %r10d
	addq	$ 32, %r13
	addq	$ 32, %rbp
	addq	$ 32, %rax

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
//	vmovupd			0(%r11), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	addq	%r12, %r11
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	0(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	0(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	0(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	0(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	0(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	0(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	0(%rax, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	0(%rax, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

	// unroll 1
//	vmovupd			0(%r11, %r12), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	addq	%r12, %r11
	vbroadcastsd	8(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	8(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	8(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	8(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	8(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	8(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	8(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	8(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	8(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	8(%rax, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	8(%rax, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

	// unroll 2
//	vmovupd			0(%r11), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	addq	%r12, %r11
	vbroadcastsd	16(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	16(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	16(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	16(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	16(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	16(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	16(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	16(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	16(%rax, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	16(%rax, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

	// unroll 3
//	vmovupd			0(%r11, %r12), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	addq	%r12, %r11
	vbroadcastsd	24(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	24(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	24(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	24(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	24(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	24(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	24(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	24(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	24(%rax, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	24(%rax, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

	subl	$ 4, %r10d
	addq	$ 32, %r13
	addq	$ 32, %rbp
	addq	$ 32, %rax

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
//	vmovupd			0(%r11), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	addq	%r12, %r11
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	0(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	0(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	0(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	0(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	0(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	0(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	0(%rax, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	0(%rax, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

	subl	$ 1, %r10d
	addq	$ 8, %r13
	addq	$ 8, %rbp
	addq	$ 8, %rax

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_4x12_vs_libcc)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- lda*sizeof(double)
// r13   <- B
// r14   <- ldb*sizeof(double)
// r15   <- km
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_4X11_VS_LIBCC
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_4x11_vs_libcc)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	vcvtsi2sd	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %ymm13
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm13, %ymm15

	leaq	0(%r14, %r14, 2), %rbx
	leaq	0(%r13, %r14, 4), %rbp
	leaq	0(%r13, %r14, 8), %rax

	// preload

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	0(%r12, %r13, 2) // software prefetch
//	prefetcht0	64(%r12, %r13, 2) // software prefetch

//	leaq	0(%r11, %r12, 4), %rbx

	// unroll 0
//	vmovupd			0(%r11), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r11, %r12, 4)
	vbroadcastsd	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%r11, %r12, 4)
	vbroadcastsd	0(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	addq	%r12, %r11
	vbroadcastsd	0(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	0(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	0(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	0(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	0(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	0(%rax, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10

	// unroll 1
//	vmovupd			0(%r11), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r11, %r12, 4)
	vbroadcastsd	8(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%r11, %r12, 4)
	vbroadcastsd	8(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	addq	%r12, %r11
	vbroadcastsd	8(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	8(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	8(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	8(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	8(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	8(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	8(%rax, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10

//	leaq	0(%r11, %r12, 2), %r11
//	leaq	0(%rbx, %r12, 2), %rbx

	// unroll 2
//	vmovupd			0(%r11), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	16(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r11)
	vbroadcastsd	16(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rbx)
	vbroadcastsd	16(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	addq	%r12, %r11
	vbroadcastsd	16(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	16(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	16(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	16(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	16(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	16(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	16(%rax, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10

	// unroll 3
//	vmovupd			0(%r11), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	24(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r11, %r12, 4)
	vbroadcastsd	24(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%r11, %r12, 4)
	vbroadcastsd	24(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	addq	%r12, %r11
	vbroadcastsd	24(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	24(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	24(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	24(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	24(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	24(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	24(%rax, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10

//	leaq	0(%r11, %r12, 2), %r11

	subl	$ 4, %r10d
	addq	$ 32, %r13
	addq	$ 32, %rbp
	addq	$ 32, %rax

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
//	vmovupd			0(%r11), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	addq	%r12, %r11
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	0(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	0(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	0(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	0(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	0(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	0(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	0(%rax, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10

	// unroll 1
//	vmovupd			0(%r11, %r12), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	addq	%r12, %r11
	vbroadcastsd	8(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	8(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	8(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	8(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	8(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	8(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	8(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	8(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	8(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	8(%rax, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10

	// unroll 2
//	vmovupd			0(%r11), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	addq	%r12, %r11
	vbroadcastsd	16(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	16(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	16(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	16(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	16(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	16(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	16(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	16(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	16(%rax, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10

	// unroll 3
//	vmovupd			0(%r11, %r12), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	addq	%r12, %r11
	vbroadcastsd	24(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	24(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	24(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	24(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	24(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	24(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	24(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	24(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	24(%rax, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10

	subl	$ 4, %r10d
	addq	$ 32, %r13
	addq	$ 32, %rbp
	addq	$ 32, %rax

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
//	vmovupd			0(%r11), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	addq	%r12, %r11
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	0(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	0(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	0(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	0(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	0(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	0(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	0(%rax, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10

	subl	$ 1, %r10d
	addq	$ 8, %r13
	addq	$ 8, %rbp
	addq	$ 8, %rax

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_4x11_vs_libcc)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- lda*sizeof(double)
// r13   <- B
// r14   <- ldb*sizeof(double)
// r15   <- km
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_4X10_VS_LIBCC
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_4x10_vs_libcc)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	vcvtsi2sd	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %ymm13
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm13, %ymm15

	leaq	0(%r14, %r14, 2), %rbx
	leaq	0(%r13, %r14, 4), %rbp
	leaq	0(%r13, %r14, 8), %rax

	// preload

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	0(%r12, %r13, 2) // software prefetch
//	prefetcht0	64(%r12, %r13, 2) // software prefetch

//	leaq	0(%r11, %r12, 4), %rbx

	// unroll 0
//	vmovupd			0(%r11), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r11, %r12, 4)
	vbroadcastsd	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%r11, %r12, 4)
	vbroadcastsd	0(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	addq	%r12, %r11
	vbroadcastsd	0(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	0(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	0(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	0(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	0(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9

	// unroll 1
//	vmovupd			0(%r11), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r11, %r12, 4)
	vbroadcastsd	8(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%r11, %r12, 4)
	vbroadcastsd	8(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	addq	%r12, %r11
	vbroadcastsd	8(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	8(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	8(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	8(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	8(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	8(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9

//	leaq	0(%r11, %r12, 2), %r11
//	leaq	0(%rbx, %r12, 2), %rbx

	// unroll 2
//	vmovupd			0(%r11), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	16(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r11)
	vbroadcastsd	16(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rbx)
	vbroadcastsd	16(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	addq	%r12, %r11
	vbroadcastsd	16(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	16(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	16(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	16(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	16(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	16(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9

	// unroll 3
//	vmovupd			0(%r11), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	24(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r11, %r12, 4)
	vbroadcastsd	24(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%r11, %r12, 4)
	vbroadcastsd	24(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	addq	%r12, %r11
	vbroadcastsd	24(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	24(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	24(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	24(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	24(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	24(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9

//	leaq	0(%r11, %r12, 2), %r11

	subl	$ 4, %r10d
	addq	$ 32, %r13
	addq	$ 32, %rbp
	addq	$ 32, %rax

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
//	vmovupd			0(%r11), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	addq	%r12, %r11
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	0(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	0(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	0(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	0(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	0(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	0(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9

	// unroll 1
//	vmovupd			0(%r11, %r12), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	addq	%r12, %r11
	vbroadcastsd	8(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	8(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	8(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	8(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	8(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	8(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	8(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	8(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	8(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9

	// unroll 2
//	vmovupd			0(%r11), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	addq	%r12, %r11
	vbroadcastsd	16(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	16(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	16(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	16(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	16(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	16(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	16(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	16(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9

	// unroll 3
//	vmovupd			0(%r11, %r12), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	addq	%r12, %r11
	vbroadcastsd	24(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	24(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	24(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	24(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	24(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	24(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	24(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	24(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9

	subl	$ 4, %r10d
	addq	$ 32, %r13
	addq	$ 32, %rbp
	addq	$ 32, %rax

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
//	vmovupd			0(%r11), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	addq	%r12, %r11
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	0(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	0(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	0(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	0(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	0(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	0(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9

	subl	$ 1, %r10d
	addq	$ 8, %r13
	addq	$ 8, %rbp
	addq	$ 8, %rax

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_4x10_vs_libcc)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- lda*sizeof(double)
// r13   <- B
// r14   <- ldb*sizeof(double)
// r15   <- km
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_4X9_VS_LIBCC
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_4x9_vs_libcc)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	vcvtsi2sd	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %ymm13
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm13, %ymm15

	leaq	0(%r14, %r14, 2), %rbx
	leaq	0(%r13, %r14, 4), %rbp
	leaq	0(%r13, %r14, 8), %rax

	// preload

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	0(%r12, %r13, 2) // software prefetch
//	prefetcht0	64(%r12, %r13, 2) // software prefetch

//	leaq	0(%r11, %r12, 4), %rbx

	// unroll 0
//	vmovupd			0(%r11), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r11, %r12, 4)
	vbroadcastsd	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%r11, %r12, 4)
	vbroadcastsd	0(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	addq	%r12, %r11
	vbroadcastsd	0(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	0(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	0(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	0(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8

	// unroll 1
//	vmovupd			0(%r11), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r11, %r12, 4)
	vbroadcastsd	8(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%r11, %r12, 4)
	vbroadcastsd	8(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	addq	%r12, %r11
	vbroadcastsd	8(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	8(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	8(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	8(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	8(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8

//	leaq	0(%r11, %r12, 2), %r11
//	leaq	0(%rbx, %r12, 2), %rbx

	// unroll 2
//	vmovupd			0(%r11), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	16(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r11)
	vbroadcastsd	16(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%rbx)
	vbroadcastsd	16(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	addq	%r12, %r11
	vbroadcastsd	16(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	16(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	16(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	16(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	16(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8

	// unroll 3
//	vmovupd			0(%r11), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	24(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r11, %r12, 4)
	vbroadcastsd	24(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
//	prefetcht0		32(%r11, %r12, 4)
	vbroadcastsd	24(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	addq	%r12, %r11
	vbroadcastsd	24(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	24(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	24(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	24(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	24(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8

//	leaq	0(%r11, %r12, 2), %r11

	subl	$ 4, %r10d
	addq	$ 32, %r13
	addq	$ 32, %rbp
	addq	$ 32, %rax

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
//	vmovupd			0(%r11), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	addq	%r12, %r11
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	0(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	0(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	0(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	0(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	0(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8

	// unroll 1
//	vmovupd			0(%r11, %r12), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	addq	%r12, %r11
	vbroadcastsd	8(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	8(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	8(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	8(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	8(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	8(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	8(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	8(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8

	// unroll 2
//	vmovupd			0(%r11), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	addq	%r12, %r11
	vbroadcastsd	16(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	16(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	16(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	16(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	16(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	16(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	16(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8

	// unroll 3
//	vmovupd			0(%r11, %r12), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	addq	%r12, %r11
	vbroadcastsd	24(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	24(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	24(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	24(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	24(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	24(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	24(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8

	subl	$ 4, %r10d
	addq	$ 32, %r13
	addq	$ 32, %rbp
	addq	$ 32, %rax

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
//	vmovupd			0(%r11), %ymm13 // A
	vmaskmovpd		0(%r11), %ymm15, %ymm13 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	addq	%r12, %r11
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	0(%r13, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	0(%rbp), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	0(%rbp, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	0(%rbp, %r14, 2), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	0(%rbp, %rbx), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8

	subl	$ 1, %r10d
	addq	$ 8, %r13
	addq	$ 8, %rbp
	addq	$ 8, %rax

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_4x9_vs_libcc)
#endif





// common inner routine with file scope
//
// edge for B lower triangular
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

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NN_RL_12X4_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nn_rl_12x4_lib4c)
#endif
	
	movq	%r13, %r15
	addq	%r14, %r15
	addq	%r14, %r15 // B+2*ldb

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vmovupd			64(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	16(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vmovupd			96(%r11, %r12), %ymm14 // A
	vmovupd			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	24(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	24(%r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r15, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl	$ 4, %r10d
	addq	$ 32, %r13
	addq	$ 32, %r15
	addq	$ 128, %r11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nn_rl_12x4_lib4c)
#endif





// common inner routine with file scope
//
// edge for B lower triangular
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

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NN_RL_12X4_VS_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nn_rl_12x4_vs_lib4c)
#endif
	
	cmp		$ 0, %r10d
	jle		0f

	movq	%r13, %r15
	addq	%r14, %r15
	addq	%r14, %r15 // B+2*ldb

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	subl	$ 1, %r10d
	addq	$ 8, %r13
	addq	$ 8, %r15
	addq	$ 32, %r11

	cmp		$ 0, %r10d
	jle		0f

	// unroll 1
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	subl	$ 1, %r10d
	addq	$ 8, %r13
	addq	$ 8, %r15
	addq	$ 32, %r11

	cmp		$ 0, %r10d
	jle		0f

	// unroll 2
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	0(%r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	subl	$ 1, %r10d
	addq	$ 8, %r13
	addq	$ 8, %r15
	addq	$ 32, %r11

	cmp		$ 0, %r10d
	jle		0f

	// unroll 3
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	0(%r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	0(%r15, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl	$ 1, %r10d
	addq	$ 8, %r13
	addq	$ 8, %r15
	addq	$ 32, %r11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nn_rl_12x4_vs_lib4c)
#endif





// common inner routine with file scope
//
// edge for B lower triangular
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

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NN_RL_ONE_12X4_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nn_rl_one_12x4_lib4c)
#endif
	
	movq	%r13, %r15
	addq	%r14, %r15
	addq	%r14, %r15 // B+2*ldb

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm0, %ymm13, %ymm0
	vaddpd			%ymm4, %ymm14, %ymm4
	vaddpd			%ymm8, %ymm15, %ymm8

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vaddpd			%ymm1, %ymm13, %ymm1
	vaddpd			%ymm5, %ymm14, %ymm5
	vaddpd			%ymm9, %ymm15, %ymm9

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vmovupd			64(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	16(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vaddpd			%ymm2, %ymm13, %ymm2
	vaddpd			%ymm6, %ymm14, %ymm6
	vaddpd			%ymm10, %ymm15, %ymm10

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vmovupd			96(%r11, %r12), %ymm14 // A
	vmovupd			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	24(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	24(%r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vaddpd			%ymm3, %ymm13, %ymm3
	vaddpd			%ymm7, %ymm14, %ymm7
	vaddpd			%ymm11, %ymm15, %ymm11

	subl	$ 4, %r10d
	addq	$ 32, %r13
	addq	$ 32, %r15
	addq	$ 128, %r11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nn_rl_one_12x4_lib4c)
#endif





// common inner routine with file scope
//
// edge for B lower triangular
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

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NN_RL_ONE_12X4_VS_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nn_rl_one_12x4_vs_lib4c)
#endif
	
	cmp		$ 0, %r10d
	jle		0f

	movq	%r13, %r15
	addq	%r14, %r15
	addq	%r14, %r15 // B+2*ldb

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm0, %ymm13, %ymm0
	vaddpd			%ymm4, %ymm14, %ymm4
	vaddpd			%ymm8, %ymm15, %ymm8

	subl	$ 1, %r10d
	addq	$ 8, %r13
	addq	$ 8, %r15
	addq	$ 32, %r11

	cmp		$ 0, %r10d
	jle		0f

	// unroll 1
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vaddpd			%ymm1, %ymm13, %ymm1
	vaddpd			%ymm5, %ymm14, %ymm5
	vaddpd			%ymm9, %ymm15, %ymm9

	subl	$ 1, %r10d
	addq	$ 8, %r13
	addq	$ 8, %r15
	addq	$ 32, %r11

	cmp		$ 0, %r10d
	jle		0f

	// unroll 2
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vaddpd			%ymm2, %ymm13, %ymm2
	vaddpd			%ymm6, %ymm14, %ymm6
	vaddpd			%ymm10, %ymm15, %ymm10

	subl	$ 1, %r10d
	addq	$ 8, %r13
	addq	$ 8, %r15
	addq	$ 32, %r11

	cmp		$ 0, %r10d
	jle		0f

	// unroll 3
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	0(%r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vaddpd			%ymm3, %ymm13, %ymm3
	vaddpd			%ymm7, %ymm14, %ymm7
	vaddpd			%ymm11, %ymm15, %ymm11

	subl	$ 1, %r10d
	addq	$ 8, %r13
	addq	$ 8, %r15
	addq	$ 32, %r11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nn_rl_one_12x4_vs_lib4c)
#endif





// common inner routine with file scope
//
// edge for B upper triangular
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
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NN_RU_12X4_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nn_ru_12x4_lib4c)
#endif
	
	movq	%r13, %r15
	addq	%r14, %r15
	addq	%r14, %r15 // B+2*ldb

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	0(%r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	0(%r15, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	8(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	8(%r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	8(%r15, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vmovupd			64(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	16(%r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	16(%r15, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vmovupd			96(%r11, %r12), %ymm14 // A
	vmovupd			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	24(%r15, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl	$ 4, %r10d
	addq	$ 32, %r13
	addq	$ 32, %r15
	addq	$ 128, %r11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nn_ru_12x4_lib4c)
#endif





// common inner routine with file scope
//
// edge for B upper triangular
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// r14   <- ldb
// r15   <- n1
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NN_RU_12X4_VS_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nn_ru_12x4_vs_lib4c)
#endif
	
	cmpl	$ 0, %r15d
	jle		0f // end

	movq	%r13, %rax
	addq	%r14, %rax
	addq	%r14, %rax // B+2*ldb

	cmpl	$ 4, %r15d
	jl		1f // end

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	0(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	8(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	8(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	8(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vmovupd			64(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	16(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	16(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vmovupd			96(%r11, %r12), %ymm14 // A
	vmovupd			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	24(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl	$ 4, %r10d
	addq	$ 32, %r13
	addq	$ 32, %rax
	addq	$ 128, %r11

	jmp		0f

1:

	cmpl	$ 3, %r15d
	jl		2f // end

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	8(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	8(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vmovupd			64(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	16(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	// unroll 3

	subl	$ 3, %r10d
	addq	$ 24, %r13
	addq	$ 24, %rax
	addq	$ 96, %r11

	jmp		0f

2:

	cmpl	$ 2, %r15d
	jl		3f // end

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	8(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	// unroll 2

	// unroll 3

	subl	$ 2, %r10d
	addq	$ 16, %r13
	addq	$ 16, %rax
	addq	$ 64, %r11

	jmp		0f

3:

//	cmpl	$ 1, %r15d
//	jl		0f // end

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	// unroll 1

	// unroll 2

	// unroll 3

	subl	$ 1, %r10d
	addq	$ 8, %r13
	addq	$ 8, %rax
	addq	$ 32, %r11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nn_ru_12x4_vs_lib4c)
#endif





// common inner routine with file scope
//
// edge for B upper triangular
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
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NN_RU_ONE_12X4_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nn_ru_one_12x4_lib4c)
#endif
	
	movq	%r13, %r15
	addq	%r14, %r15
	addq	%r14, %r15 // B+2*ldb

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm0, %ymm13, %ymm0
	vaddpd			%ymm4, %ymm14, %ymm4
	vaddpd			%ymm8, %ymm15, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	0(%r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	0(%r15, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			32(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm1, %ymm13, %ymm1
	vaddpd			%ymm5, %ymm14, %ymm5
	vaddpd			%ymm9, %ymm15, %ymm9
	vbroadcastsd	8(%r15), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	8(%r15, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vmovupd			64(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm2, %ymm13, %ymm2
	vaddpd			%ymm6, %ymm14, %ymm6
	vaddpd			%ymm10, %ymm15, %ymm10
	vbroadcastsd	16(%r15, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vmovupd			96(%r11, %r12), %ymm14 // A
	vmovupd			96(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm3, %ymm13, %ymm3
	vaddpd			%ymm7, %ymm14, %ymm7
	vaddpd			%ymm11, %ymm15, %ymm11

	subl	$ 4, %r10d
	addq	$ 32, %r13
	addq	$ 32, %r15
	addq	$ 128, %r11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nn_ru_one_12x4_lib4c)
#endif





// common inner routine with file scope
//
// edge for B upper triangular
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// r14   <- ldb
// r15   <- n1
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NN_RU_ONE_12X4_VS_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nn_ru_one_12x4_vs_lib4c)
#endif
	
	cmpl	$ 0, %r15d
	jle		0f // end

	movq	%r13, %rax
	addq	%r14, %rax
	addq	%r14, %rax // B+2*ldb

	cmpl	$ 4, %r15d
	jl		1f // end

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm0, %ymm13, %ymm0
	vaddpd			%ymm4, %ymm14, %ymm4
	vaddpd			%ymm8, %ymm15, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	0(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			32(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm1, %ymm13, %ymm1
	vaddpd			%ymm5, %ymm14, %ymm5
	vaddpd			%ymm9, %ymm15, %ymm9
	vbroadcastsd	8(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	8(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vmovupd			64(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm2, %ymm13, %ymm2
	vaddpd			%ymm6, %ymm14, %ymm6
	vaddpd			%ymm10, %ymm15, %ymm10
	vbroadcastsd	16(%rax, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovupd			96(%r11), %ymm13 // A
	vmovupd			96(%r11, %r12), %ymm14 // A
	vmovupd			96(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm3, %ymm13, %ymm3
	vaddpd			%ymm7, %ymm14, %ymm7
	vaddpd			%ymm11, %ymm15, %ymm11

	subl	$ 4, %r10d
	addq	$ 32, %r13
	addq	$ 32, %rax
	addq	$ 128, %r11

	jmp		0f

1:

	cmpl	$ 3, %r15d
	jl		2f // end

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm0, %ymm13, %ymm0
	vaddpd			%ymm4, %ymm14, %ymm4
	vaddpd			%ymm8, %ymm15, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	0(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			32(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm1, %ymm13, %ymm1
	vaddpd			%ymm5, %ymm14, %ymm5
	vaddpd			%ymm9, %ymm15, %ymm9
	vbroadcastsd	8(%rax), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	// unroll 2
	vmovupd			64(%r11), %ymm13 // A
	vmovupd			64(%r11, %r12), %ymm14 // A
	vmovupd			64(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm2, %ymm13, %ymm2
	vaddpd			%ymm6, %ymm14, %ymm6
	vaddpd			%ymm10, %ymm15, %ymm10

	// unroll 3

	subl	$ 3, %r10d
	addq	$ 24, %r13
	addq	$ 24, %rax
	addq	$ 96, %r11

	jmp		0f

2:

	cmpl	$ 2, %r15d
	jl		3f // end

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm0, %ymm13, %ymm0
	vaddpd			%ymm4, %ymm14, %ymm4
	vaddpd			%ymm8, %ymm15, %ymm8
	vbroadcastsd	0(%r13, %r14), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	// unroll 1
	vmovupd			32(%r11), %ymm13 // A
	vmovupd			32(%r11, %r12), %ymm14 // A
	vmovupd			32(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm1, %ymm13, %ymm1
	vaddpd			%ymm5, %ymm14, %ymm5
	vaddpd			%ymm9, %ymm15, %ymm9

	// unroll 2

	// unroll 3

	subl	$ 2, %r10d
	addq	$ 16, %r13
	addq	$ 16, %rax
	addq	$ 64, %r11

	jmp		0f

3:

//	cmpl	$ 1, %r15d
//	jl		0f // end

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vmovupd			0(%r11, %r12), %ymm14 // A
	vmovupd			0(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm0, %ymm13, %ymm0
	vaddpd			%ymm4, %ymm14, %ymm4
	vaddpd			%ymm8, %ymm15, %ymm8

	// unroll 1

	// unroll 2

	// unroll 3

	subl	$ 1, %r10d
	addq	$ 8, %r13
	addq	$ 8, %rax
	addq	$ 32, %r11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nn_ru_one_12x4_vs_lib4c)
#endif





// common inner routine with file scope
//
// edge for B lower triangular
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
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RL_12X4_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_rl_12x4_lib4c)
#endif
	
	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vmovapd			0(%r11, %r12), %ymm14 // A
	vmovapd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vmovapd			32(%r11, %r12), %ymm14 // A
	vmovapd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 2
	vmovapd			64(%r11), %ymm13 // A
	vmovapd			64(%r11, %r12), %ymm14 // A
	vmovapd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 3
	vmovapd			96(%r11), %ymm13 // A
	vmovapd			96(%r11, %r12), %ymm14 // A
	vmovapd			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 128, %r11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_rl_12x4_lib4c)
#endif





// common inner routine with file scope
//
// edge for B upper triangular
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// r14   <- ldb
// r15   <- n1
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RL_12X4_VS_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_rl_12x4_vs_lib4c)
#endif
	
	cmpl	$ 0, %r15d
	jle		0f // end

	cmpl	$ 4, %r15d
	jl		1f // end

	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vmovapd			0(%r11, %r12), %ymm14 // A
	vmovapd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vmovapd			32(%r11, %r12), %ymm14 // A
	vmovapd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 2
	vmovapd			64(%r11), %ymm13 // A
	vmovapd			64(%r11, %r12), %ymm14 // A
	vmovapd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 3
	vmovapd			96(%r11), %ymm13 // A
	vmovapd			96(%r11, %r12), %ymm14 // A
	vmovapd			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 128, %r11

	jmp		0f

1:

	cmpl	$ 3, %r15d
	jl		2f // end

	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vmovapd			0(%r11, %r12), %ymm14 // A
	vmovapd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	addq	%r14, %r13

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vmovapd			32(%r11, %r12), %ymm14 // A
	vmovapd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	addq	%r14, %r13

	// unroll 2
	vmovapd			64(%r11), %ymm13 // A
	vmovapd			64(%r11, %r12), %ymm14 // A
	vmovapd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	addq	%r14, %r13

	// unroll 3

	subl	$ 3, %r10d
	addq	$ 96, %r11

	jmp		0f

2:

	cmpl	$ 2, %r15d
	jl		3f // end

	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vmovapd			0(%r11, %r12), %ymm14 // A
	vmovapd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	addq	%r14, %r13

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vmovapd			32(%r11, %r12), %ymm14 // A
	vmovapd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	addq	%r14, %r13

	// unroll 2

	// unroll 3

	subl	$ 2, %r10d
	addq	$ 64, %r11

	jmp		0f

3:

//	cmpl	$ 1, %r15d
//	jl		0f // end

	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vmovapd			0(%r11, %r12), %ymm14 // A
	vmovapd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	addq	%r14, %r13

	// unroll 1

	// unroll 2

	// unroll 3

	subl	$ 1, %r10d
	addq	$ 32, %r11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_rl_12x4_vs_lib4c)
#endif





// common inner routine with file scope
//
// edge for B lower triangular
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
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RL_ONE_12X4_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_rl_one_12x4_lib4c)
#endif
	
	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vmovapd			0(%r11, %r12), %ymm14 // A
	vmovapd			0(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm0, %ymm13, %ymm0
	vaddpd			%ymm4, %ymm14, %ymm4
	vaddpd			%ymm8, %ymm15, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vmovapd			32(%r11, %r12), %ymm14 // A
	vmovapd			32(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm1, %ymm13, %ymm1
	vaddpd			%ymm5, %ymm14, %ymm5
	vaddpd			%ymm9, %ymm15, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 2
	vmovapd			64(%r11), %ymm13 // A
	vmovapd			64(%r11, %r12), %ymm14 // A
	vmovapd			64(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm2, %ymm13, %ymm2
	vaddpd			%ymm6, %ymm14, %ymm6
	vaddpd			%ymm10, %ymm15, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 3
	vmovapd			96(%r11), %ymm13 // A
	vmovapd			96(%r11, %r12), %ymm14 // A
	vmovapd			96(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm3, %ymm13, %ymm3
	vaddpd			%ymm7, %ymm14, %ymm7
	vaddpd			%ymm11, %ymm15, %ymm11
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 128, %r11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_rl_one_12x4_lib4c)
#endif





// common inner routine with file scope
//
// edge for B upper triangular
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// r14   <- ldb
// r15   <- n1
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RL_ONE_12X4_VS_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_rl_one_12x4_vs_lib4c)
#endif
	
	cmpl	$ 0, %r15d
	jle		0f // end

	cmpl	$ 4, %r15d
	jl		1f // end

	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vmovapd			0(%r11, %r12), %ymm14 // A
	vmovapd			0(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm0, %ymm13, %ymm0
	vaddpd			%ymm4, %ymm14, %ymm4
	vaddpd			%ymm8, %ymm15, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vmovapd			32(%r11, %r12), %ymm14 // A
	vmovapd			32(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm1, %ymm13, %ymm1
	vaddpd			%ymm5, %ymm14, %ymm5
	vaddpd			%ymm9, %ymm15, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 2
	vmovapd			64(%r11), %ymm13 // A
	vmovapd			64(%r11, %r12), %ymm14 // A
	vmovapd			64(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm2, %ymm13, %ymm2
	vaddpd			%ymm6, %ymm14, %ymm6
	vaddpd			%ymm10, %ymm15, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 3
	vmovapd			96(%r11), %ymm13 // A
	vmovapd			96(%r11, %r12), %ymm14 // A
	vmovapd			96(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm3, %ymm13, %ymm3
	vaddpd			%ymm7, %ymm14, %ymm7
	vaddpd			%ymm11, %ymm15, %ymm11
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 128, %r11

	jmp		0f

1:

	cmpl	$ 3, %r15d
	jl		2f // end

	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vmovapd			0(%r11, %r12), %ymm14 // A
	vmovapd			0(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm0, %ymm13, %ymm0
	vaddpd			%ymm4, %ymm14, %ymm4
	vaddpd			%ymm8, %ymm15, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	addq	%r14, %r13

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vmovapd			32(%r11, %r12), %ymm14 // A
	vmovapd			32(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm1, %ymm13, %ymm1
	vaddpd			%ymm5, %ymm14, %ymm5
	vaddpd			%ymm9, %ymm15, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	addq	%r14, %r13

	// unroll 2
	vmovapd			64(%r11), %ymm13 // A
	vmovapd			64(%r11, %r12), %ymm14 // A
	vmovapd			64(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm2, %ymm13, %ymm2
	vaddpd			%ymm6, %ymm14, %ymm6
	vaddpd			%ymm10, %ymm15, %ymm10
	addq	%r14, %r13

	// unroll 3

	subl	$ 3, %r10d
	addq	$ 96, %r11

	jmp		0f

2:

	cmpl	$ 2, %r15d
	jl		3f // end

	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vmovapd			0(%r11, %r12), %ymm14 // A
	vmovapd			0(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm0, %ymm13, %ymm0
	vaddpd			%ymm4, %ymm14, %ymm4
	vaddpd			%ymm8, %ymm15, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	addq	%r14, %r13

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vmovapd			32(%r11, %r12), %ymm14 // A
	vmovapd			32(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm1, %ymm13, %ymm1
	vaddpd			%ymm5, %ymm14, %ymm5
	vaddpd			%ymm9, %ymm15, %ymm9
	addq	%r14, %r13

	// unroll 2

	// unroll 3

	subl	$ 2, %r10d
	addq	$ 64, %r11

	jmp		0f

3:

//	cmpl	$ 1, %r15d
//	jl		0f // end

	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vmovapd			0(%r11, %r12), %ymm14 // A
	vmovapd			0(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm0, %ymm13, %ymm0
	vaddpd			%ymm4, %ymm14, %ymm4
	vaddpd			%ymm8, %ymm15, %ymm8
	addq	%r14, %r13

	// unroll 1

	// unroll 2

	// unroll 3

	subl	$ 1, %r10d
	addq	$ 32, %r11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_rl_one_12x4_vs_lib4c)
#endif





// common inner routine with file scope
//
// edge for B upper triangular
//
// input arguments:
// r10   <- kmax
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
// ymm8  <- [d80 d90 da0 db0]
// ymm9  <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]

//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RU_12X4_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_ru_12x4_lib4c)
#endif
	
	vbroadcastsd	0(%r13), %ymm12
	vmovapd			0(%r11), %ymm13
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			0(%r11, %r12), %ymm14
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			0(%r11, %r12, 2), %ymm15
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	addq			%r14, %r13

	vbroadcastsd	0(%r13), %ymm12
	vmovapd			32(%r11), %ymm13
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			32(%r11, %r12), %ymm14
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			32(%r11, %r12, 2), %ymm15
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	addq			%r14, %r13

	vbroadcastsd	0(%r13), %ymm12
	vmovapd			64(%r11), %ymm13
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			64(%r11, %r12), %ymm14
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			64(%r11, %r12, 2), %ymm15
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	addq			%r14, %r13

	vbroadcastsd	0(%r13), %ymm12
	vmovapd			96(%r11), %ymm13
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			96(%r11, %r12), %ymm14
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			96(%r11, %r12, 2), %ymm15
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq			%r14, %r13

	subl	$ 4, %r10d
	addq	$ 128, %r11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_ru_12x4_lib4c)
#endif





// common inner routine with file scope
//
// edge for B upper triangular
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
// ymm8  <- [d80 d90 da0 db0]
// ymm9  <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]

//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RU_12X4_VS_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_ru_12x4_vs_lib4c)
#endif
	
	cmp		$ 0, %r10d
	jle		0f

	vbroadcastsd	0(%r13), %ymm12
	vmovapd			0(%r11), %ymm13
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			0(%r11, %r12), %ymm14
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			0(%r11, %r12, 2), %ymm15
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	subl			$ 1, %r10d
	addq			$ 32, %r11
	addq			%r14, %r13

	cmpl	$ 0, %r10d
	jle		0f

	vbroadcastsd	0(%r13), %ymm12
	vmovapd			0(%r11), %ymm13
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			0(%r11, %r12), %ymm14
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			0(%r11, %r12, 2), %ymm15
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	subl			$ 1, %r10d
	addq			$ 32, %r11
	addq			%r14, %r13

	cmpl	$ 0, %r10d
	jle		0f

	vbroadcastsd	0(%r13), %ymm12
	vmovapd			0(%r11), %ymm13
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			0(%r11, %r12), %ymm14
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			0(%r11, %r12, 2), %ymm15
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	subl			$ 1, %r10d
	addq			$ 32, %r11
	addq			%r14, %r13

	cmpl	$ 0, %r10d
	jle		0f

	vbroadcastsd	0(%r13), %ymm12
	vmovapd			0(%r11), %ymm13
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			0(%r11, %r12), %ymm14
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			0(%r11, %r12, 2), %ymm15
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	subl			$ 1, %r10d
	addq			$ 32, %r11
	addq			%r14, %r13

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_ru_12x4_vs_lib4c)
#endif





// common inner routine with file scope
//
// edge for B upper triangular
//
// input arguments:
// r10   <- kmax
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
// ymm8  <- [d80 d90 da0 db0]
// ymm9  <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]

//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RU_ONE_12X4_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_ru_one_12x4_lib4c)
#endif
	
	vmovapd			0(%r11), %ymm13
	vaddpd			%ymm0, %ymm13, %ymm0
	vmovapd			0(%r11, %r12), %ymm14
	vaddpd			%ymm4, %ymm14, %ymm4
	vmovapd			0(%r11, %r12, 2), %ymm15
	vaddpd			%ymm8, %ymm15, %ymm8
	addq			%r14, %r13

	vbroadcastsd	0(%r13), %ymm12
	vmovapd			32(%r11), %ymm13
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			32(%r11, %r12), %ymm14
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			32(%r11, %r12, 2), %ymm15
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vaddpd			%ymm1, %ymm13, %ymm1
	vaddpd			%ymm5, %ymm14, %ymm5
	vaddpd			%ymm9, %ymm15, %ymm9
	addq			%r14, %r13

	vbroadcastsd	0(%r13), %ymm12
	vmovapd			64(%r11), %ymm13
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			64(%r11, %r12), %ymm14
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			64(%r11, %r12, 2), %ymm15
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vaddpd			%ymm2, %ymm13, %ymm2
	vaddpd			%ymm6, %ymm14, %ymm6
	vaddpd			%ymm10, %ymm15, %ymm10
	addq			%r14, %r13

	vbroadcastsd	0(%r13), %ymm12
	vmovapd			96(%r11), %ymm13
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			96(%r11, %r12), %ymm14
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			96(%r11, %r12, 2), %ymm15
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vaddpd			%ymm3, %ymm13, %ymm3
	vaddpd			%ymm7, %ymm14, %ymm7
	vaddpd			%ymm11, %ymm15, %ymm11
	addq			%r14, %r13

	subl	$ 4, %r10d
	addq	$ 128, %r11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_ru_one_12x4_lib4c)
#endif





// common inner routine with file scope
//
// edge for B upper triangular
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
// ymm8  <- [d80 d90 da0 db0]
// ymm9  <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]

//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RU_ONE_12X4_VS_LIB4C
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_ru_one_12x4_vs_lib4c)
#endif
	
	cmp		$ 0, %r10d
	jle		0f

	vmovapd			0(%r11), %ymm13
	vaddpd			%ymm0, %ymm13, %ymm0
	vmovapd			0(%r11, %r12), %ymm14
	vaddpd			%ymm4, %ymm14, %ymm4
	vmovapd			0(%r11, %r12, 2), %ymm15
	vaddpd			%ymm8, %ymm15, %ymm8
	subl			$ 1, %r10d
	addq			$ 32, %r11
	addq			%r14, %r13

	cmpl	$ 0, %r10d
	jle		0f

	vbroadcastsd	0(%r13), %ymm12
	vmovapd			0(%r11), %ymm13
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			0(%r11, %r12), %ymm14
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			0(%r11, %r12, 2), %ymm15
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vaddpd			%ymm1, %ymm13, %ymm1
	vaddpd			%ymm5, %ymm14, %ymm5
	vaddpd			%ymm9, %ymm15, %ymm9
	subl			$ 1, %r10d
	addq			$ 32, %r11
	addq			%r14, %r13

	cmpl	$ 0, %r10d
	jle		0f

	vbroadcastsd	0(%r13), %ymm12
	vmovapd			0(%r11), %ymm13
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			0(%r11, %r12), %ymm14
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			0(%r11, %r12, 2), %ymm15
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vaddpd			%ymm2, %ymm13, %ymm2
	vaddpd			%ymm6, %ymm14, %ymm6
	vaddpd			%ymm10, %ymm15, %ymm10
	subl			$ 1, %r10d
	addq			$ 32, %r11
	addq			%r14, %r13

	cmpl	$ 0, %r10d
	jle		0f

	vbroadcastsd	0(%r13), %ymm12
	vmovapd			0(%r11), %ymm13
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			0(%r11, %r12), %ymm14
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			0(%r11, %r12, 2), %ymm15
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vaddpd			%ymm3, %ymm13, %ymm3
	vaddpd			%ymm7, %ymm14, %ymm7
	vaddpd			%ymm11, %ymm15, %ymm11
	subl			$ 1, %r10d
	addq			$ 32, %r11
	addq			%r14, %r13

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_ru_one_12x4_vs_lib4c)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = left
// uplo = lower
// tran = not-transposed
// unit diagonal
//
// input arguments:
// r10  <- E
// r11  <- lde
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_LLN_ONE_12X4_LIBC
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_lln_one_12x4_libc)
#endif
	
	// left block-column

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovupd			0(%r10), %ymm12
	vblendpd		$ 0x1, %ymm15, %ymm12, %ymm12
	vmovupd			32(%r10), %ymm14
	vmovupd			64(%r10), %ymm15
	vpermpd			$ 0x00, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0x00, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0x00, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0x00, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11
	add				%r11, %r10

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovupd			0(%r10), %ymm12
	vblendpd		$ 0x3, %ymm15, %ymm12, %ymm12
	vmovupd			32(%r10), %ymm14
	vmovupd			64(%r10), %ymm15
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0x55, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0x55, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0x55, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11
	add				%r11, %r10

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovupd			0(%r10), %ymm12
	vblendpd		$ 0x7, %ymm15, %ymm12, %ymm12
	vmovupd			32(%r10), %ymm14
	vmovupd			64(%r10), %ymm15
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0xaa, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0xaa, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0xaa, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11
	add				%r11, %r10

	vmovupd			32(%r10), %ymm14
	vmovupd			64(%r10), %ymm15
	vpermpd			$ 0xff, %ymm0, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0xff, %ymm1, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0xff, %ymm2, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0xff, %ymm3, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11
	add				%r11, %r10


	// middle block-column

	vxorpd			%ymm14, %ymm14, %ymm14
	vmovupd			32(%r10), %ymm12
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm12
	vmovupd			64(%r10), %ymm14
	vpermpd			$ 0x00, %ymm4, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm4
	vfnmadd231pd	%ymm14, %ymm13, %ymm8
	vpermpd			$ 0x00, %ymm5, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm5
	vfnmadd231pd	%ymm14, %ymm13, %ymm9
	vpermpd			$ 0x00, %ymm6, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm6
	vfnmadd231pd	%ymm14, %ymm13, %ymm10
	vpermpd			$ 0x00, %ymm7, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm7
	vfnmadd231pd	%ymm14, %ymm13, %ymm11
	add				%r11, %r10

	vxorpd			%ymm14, %ymm14, %ymm14
	vmovupd			32(%r10), %ymm12
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm12
	vmovupd			64(%r10), %ymm14
	vpermpd			$ 0x55, %ymm4, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm4
	vfnmadd231pd	%ymm14, %ymm13, %ymm8
	vpermpd			$ 0x55, %ymm5, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm5
	vfnmadd231pd	%ymm14, %ymm13, %ymm9
	vpermpd			$ 0x55, %ymm6, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm6
	vfnmadd231pd	%ymm14, %ymm13, %ymm10
	vpermpd			$ 0x55, %ymm7, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm7
	vfnmadd231pd	%ymm14, %ymm13, %ymm11
	add				%r11, %r10

	vxorpd			%ymm14, %ymm14, %ymm14
	vmovupd			32(%r10), %ymm12
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm12
	vmovupd			64(%r10), %ymm14
	vpermpd			$ 0xaa, %ymm4, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm4
	vfnmadd231pd	%ymm14, %ymm13, %ymm8
	vpermpd			$ 0xaa, %ymm5, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm5
	vfnmadd231pd	%ymm14, %ymm13, %ymm9
	vpermpd			$ 0xaa, %ymm6, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm6
	vfnmadd231pd	%ymm14, %ymm13, %ymm10
	vpermpd			$ 0xaa, %ymm7, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm7
	vfnmadd231pd	%ymm14, %ymm13, %ymm11
	add				%r11, %r10

	vmovupd			64(%r10), %ymm14
	vpermpd			$ 0xff, %ymm4, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm8
	vpermpd			$ 0xff, %ymm5, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm9
	vpermpd			$ 0xff, %ymm6, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm10
	vpermpd			$ 0xff, %ymm7, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm11
	add				%r11, %r10


	// right block-column

	vxorpd			%ymm14, %ymm14, %ymm14

	vmovupd			64(%r10), %ymm12
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm12
	vpermpd			$ 0x00, %ymm8, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm8
	vpermpd			$ 0x00, %ymm9, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm9
	vpermpd			$ 0x00, %ymm10, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm10
	vpermpd			$ 0x00, %ymm11, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm11
	add				%r11, %r10

	vmovupd			64(%r10), %ymm12
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm12
	vpermpd			$ 0x55, %ymm8, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm8
	vpermpd			$ 0x55, %ymm9, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm9
	vpermpd			$ 0x55, %ymm10, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm10
	vpermpd			$ 0x55, %ymm11, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm11
	add				%r11, %r10

	vmovupd			64(%r10), %ymm12
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm12
	vpermpd			$ 0xaa, %ymm8, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm8
	vpermpd			$ 0xaa, %ymm9, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm9
	vpermpd			$ 0xaa, %ymm10, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm10
	vpermpd			$ 0xaa, %ymm11, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_lln_one_12x4_libc)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = left
// uplo = lower
// tran = not-transposed
// unit diagonal
//
// input arguments:
// r10  <- E
// r11  <- lde
// r12  <- m1
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_LLN_ONE_12X4_VS_LIBC
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_lln_one_12x4_vs_libc)
#endif
	
	// left block-column


	vxorpd			%ymm15, %ymm15, %ymm15
	vmovupd			0(%r10), %ymm12
	vblendpd		$ 0x1, %ymm15, %ymm12, %ymm12
	vcvtsi2sd	%r12d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC04(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC04(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15
	vmaskmovpd		64(%r10), %ymm15, %ymm15
	vmovupd			32(%r10), %ymm14
	vpermpd			$ 0x00, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0x00, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0x00, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0x00, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11
	add				%r11, %r10

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovupd			0(%r10), %ymm12
	vblendpd		$ 0x3, %ymm15, %ymm12, %ymm12
	vcvtsi2sd	%r12d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC04(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC04(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15
	vmaskmovpd		64(%r10), %ymm15, %ymm15
	vmovupd			32(%r10), %ymm14
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0x55, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0x55, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0x55, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11
	add				%r11, %r10

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovupd			0(%r10), %ymm12
	vblendpd		$ 0x7, %ymm15, %ymm12, %ymm12
	vcvtsi2sd	%r12d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC04(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC04(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15
	vmaskmovpd		64(%r10), %ymm15, %ymm15
	vmovupd			32(%r10), %ymm14
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0xaa, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0xaa, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0xaa, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11
	add				%r11, %r10

	vmovupd			32(%r10), %ymm12
	vcvtsi2sd	%r12d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC04(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC04(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15
	vmaskmovpd		64(%r10), %ymm15, %ymm14
	vpermpd			$ 0xff, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm4
	vfnmadd231pd	%ymm14, %ymm13, %ymm8
	vpermpd			$ 0xff, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm5
	vfnmadd231pd	%ymm14, %ymm13, %ymm9
	vpermpd			$ 0xff, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm6
	vfnmadd231pd	%ymm14, %ymm13, %ymm10
	vpermpd			$ 0xff, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm7
	vfnmadd231pd	%ymm14, %ymm13, %ymm11
	add				%r11, %r10


	// middle block-column

	vxorpd			%ymm14, %ymm14, %ymm14
	vmovupd			32(%r10), %ymm12
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm12
	vmaskmovpd		64(%r10), %ymm15, %ymm14
	vpermpd			$ 0x00, %ymm4, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm4
	vfnmadd231pd	%ymm14, %ymm13, %ymm8
	vpermpd			$ 0x00, %ymm5, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm5
	vfnmadd231pd	%ymm14, %ymm13, %ymm9
	vpermpd			$ 0x00, %ymm6, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm6
	vfnmadd231pd	%ymm14, %ymm13, %ymm10
	vpermpd			$ 0x00, %ymm7, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm7
	vfnmadd231pd	%ymm14, %ymm13, %ymm11
	add				%r11, %r10

	vxorpd			%ymm14, %ymm14, %ymm14
	vmovupd			32(%r10), %ymm12
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm12
	vmaskmovpd		64(%r10), %ymm15, %ymm14
	vpermpd			$ 0x55, %ymm4, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm4
	vfnmadd231pd	%ymm14, %ymm13, %ymm8
	vpermpd			$ 0x55, %ymm5, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm5
	vfnmadd231pd	%ymm14, %ymm13, %ymm9
	vpermpd			$ 0x55, %ymm6, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm6
	vfnmadd231pd	%ymm14, %ymm13, %ymm10
	vpermpd			$ 0x55, %ymm7, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm7
	vfnmadd231pd	%ymm14, %ymm13, %ymm11
	add				%r11, %r10

	vxorpd			%ymm14, %ymm14, %ymm14
	vmovupd			32(%r10), %ymm12
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm12
	vmaskmovpd		64(%r10), %ymm15, %ymm14
	vpermpd			$ 0xaa, %ymm4, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm4
	vfnmadd231pd	%ymm14, %ymm13, %ymm8
	vpermpd			$ 0xaa, %ymm5, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm5
	vfnmadd231pd	%ymm14, %ymm13, %ymm9
	vpermpd			$ 0xaa, %ymm6, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm6
	vfnmadd231pd	%ymm14, %ymm13, %ymm10
	vpermpd			$ 0xaa, %ymm7, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm7
	vfnmadd231pd	%ymm14, %ymm13, %ymm11
	add				%r11, %r10

	vmaskmovpd		64(%r10), %ymm15, %ymm14
	vpermpd			$ 0xff, %ymm4, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm8
	vpermpd			$ 0xff, %ymm5, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm9
	vpermpd			$ 0xff, %ymm6, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm10
	vpermpd			$ 0xff, %ymm7, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm11
	add				%r11, %r10


	// right block-column

	cmpl			$ 9, %r12d
	jle				0f

	vxorpd			%ymm14, %ymm14, %ymm14

	vmaskmovpd		64(%r10), %ymm15, %ymm12
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm12
	vpermpd			$ 0x00, %ymm8, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm8
	vpermpd			$ 0x00, %ymm9, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm9
	vpermpd			$ 0x00, %ymm10, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm10
	vpermpd			$ 0x00, %ymm11, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm11
	add				%r11, %r10

	cmpl			$ 10, %r12d
	jle				0f

	vmaskmovpd		64(%r10), %ymm15, %ymm12
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm12
	vpermpd			$ 0x55, %ymm8, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm8
	vpermpd			$ 0x55, %ymm9, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm9
	vpermpd			$ 0x55, %ymm10, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm10
	vpermpd			$ 0x55, %ymm11, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm11
	add				%r11, %r10

	cmpl			$ 11, %r12d
	jle				0f

	vmaskmovpd		64(%r10), %ymm15, %ymm12
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm12
	vpermpd			$ 0xaa, %ymm8, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm8
	vpermpd			$ 0xaa, %ymm9, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm9
	vpermpd			$ 0xaa, %ymm10, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm10
	vpermpd			$ 0xaa, %ymm11, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_lln_one_12x4_vs_libc)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = lower
// tran = not-transposed
// requires explicit inverse of diagonal
//
// input arguments:
// r10  <- E
// r11  <- lde
// r12  <- inv_diag_E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- []
// ymm5 <- []
// ymm6 <- []
// ymm7 <- []
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLN_INV_12X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rln_inv_12x4_lib)
#endif
	
	// 4th column
	vbroadcastsd	24(%r12), %ymm13
	vmulpd			%ymm3, %ymm13, %ymm3
	vmulpd			%ymm7, %ymm13, %ymm7
	vmulpd			%ymm11, %ymm13, %ymm11
	vbroadcastsd	24(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm2
	vfnmadd231pd	%ymm7, %ymm13, %ymm6
	vfnmadd231pd	%ymm11, %ymm13, %ymm10
	vbroadcastsd	24(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm1
	vfnmadd231pd	%ymm7, %ymm13, %ymm5
	vfnmadd231pd	%ymm11, %ymm13, %ymm9
	vbroadcastsd	24(%r10), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm0
	vfnmadd231pd	%ymm7, %ymm13, %ymm4
	vfnmadd231pd	%ymm11, %ymm13, %ymm8

	// 3rd column
	vbroadcastsd	16(%r12), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm2
	vmulpd			%ymm6, %ymm13, %ymm6
	vmulpd			%ymm10, %ymm13, %ymm10
	vbroadcastsd	16(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm1
	vfnmadd231pd	%ymm6, %ymm13, %ymm5
	vfnmadd231pd	%ymm10, %ymm13, %ymm9
	vbroadcastsd	16(%r10), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm0
	vfnmadd231pd	%ymm6, %ymm13, %ymm4
	vfnmadd231pd	%ymm10, %ymm13, %ymm8

	// 2nd column
	vbroadcastsd	8(%r12), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm1
	vmulpd			%ymm5, %ymm13, %ymm5
	vmulpd			%ymm9, %ymm13, %ymm9
	vbroadcastsd	8(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm0
	vfnmadd231pd	%ymm5, %ymm13, %ymm4
	vfnmadd231pd	%ymm9, %ymm13, %ymm8

	// 1st column
	vbroadcastsd	0(%r12), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm0
	vmulpd			%ymm4, %ymm13, %ymm4
	vmulpd			%ymm8, %ymm13, %ymm8

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rln_inv_12x4_lib)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = lower
// tran = not-transposed
// requires explicit inverse of diagonal
//
// input arguments:
// r10  <- E
// r11  <- lde
// r12  <- inv_diag_E
// r13  <- n1
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- []
// ymm5 <- []
// ymm6 <- []
// ymm7 <- []
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLN_INV_12X4_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rln_inv_12x4_vs_lib)
#endif
	
	cmpl			$ 3, %r13d
	jle				0f

	// 4th column
	vbroadcastsd	24(%r12), %ymm13
	vmulpd			%ymm3, %ymm13, %ymm3
	vmulpd			%ymm7, %ymm13, %ymm7
	vmulpd			%ymm11, %ymm13, %ymm11
	vbroadcastsd	24(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm2
	vfnmadd231pd	%ymm7, %ymm13, %ymm6
	vfnmadd231pd	%ymm11, %ymm13, %ymm10
	vbroadcastsd	24(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm1
	vfnmadd231pd	%ymm7, %ymm13, %ymm5
	vfnmadd231pd	%ymm11, %ymm13, %ymm9
	vbroadcastsd	24(%r10), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm0
	vfnmadd231pd	%ymm7, %ymm13, %ymm4
	vfnmadd231pd	%ymm11, %ymm13, %ymm8

0:
	cmpl			$ 2, %r13d
	jle				0f

	// 3rd column
	vbroadcastsd	16(%r12), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm2
	vmulpd			%ymm6, %ymm13, %ymm6
	vmulpd			%ymm10, %ymm13, %ymm10
	vbroadcastsd	16(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm1
	vfnmadd231pd	%ymm6, %ymm13, %ymm5
	vfnmadd231pd	%ymm10, %ymm13, %ymm9
	vbroadcastsd	16(%r10), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm0
	vfnmadd231pd	%ymm6, %ymm13, %ymm4
	vfnmadd231pd	%ymm10, %ymm13, %ymm8

0:
	cmpl			$ 1, %r13d
	jle				0f

	// 2nd column
	vbroadcastsd	8(%r12), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm1
	vmulpd			%ymm5, %ymm13, %ymm5
	vmulpd			%ymm9, %ymm13, %ymm9
	vbroadcastsd	8(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm0
	vfnmadd231pd	%ymm5, %ymm13, %ymm4
	vfnmadd231pd	%ymm9, %ymm13, %ymm8

0:

	// 1st column
	vbroadcastsd	0(%r12), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm0
	vmulpd			%ymm4, %ymm13, %ymm4
	vmulpd			%ymm8, %ymm13, %ymm8

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rln_inv_12x4_vs_lib)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = lower
// tran = not-transposed
// unit diagonal
//
// input arguments:
// r10  <- E
// r11  <- lde
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- []
// ymm5 <- []
// ymm6 <- []
// ymm7 <- []
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLN_ONE_12X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rln_one_12x4_lib)
#endif
	
	// 4th column
	vbroadcastsd	24(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm2
	vfnmadd231pd	%ymm7, %ymm13, %ymm6
	vfnmadd231pd	%ymm11, %ymm13, %ymm10
	vbroadcastsd	24(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm1
	vfnmadd231pd	%ymm7, %ymm13, %ymm5
	vfnmadd231pd	%ymm11, %ymm13, %ymm9
	vbroadcastsd	24(%r10), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm0
	vfnmadd231pd	%ymm7, %ymm13, %ymm4
	vfnmadd231pd	%ymm11, %ymm13, %ymm8

	// 3rd column
	vbroadcastsd	16(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm1
	vfnmadd231pd	%ymm6, %ymm13, %ymm5
	vfnmadd231pd	%ymm10, %ymm13, %ymm9
	vbroadcastsd	16(%r10), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm0
	vfnmadd231pd	%ymm6, %ymm13, %ymm4
	vfnmadd231pd	%ymm10, %ymm13, %ymm8

	// 2nd column
	vbroadcastsd	8(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm0
	vfnmadd231pd	%ymm5, %ymm13, %ymm4
	vfnmadd231pd	%ymm9, %ymm13, %ymm8

	// 1st column

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rln_one_12x4_lib)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = lower
// tran = not-transposed
// unit diagonal
//
// input arguments:
// r10  <- E
// r11  <- lde
// r12  <- n1
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- []
// ymm5 <- []
// ymm6 <- []
// ymm7 <- []
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLN_ONE_12X4_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rln_one_12x4_vs_lib)
#endif
	
	cmpl			$ 3, %r12d
	jle				0f

	// 4th column
	vbroadcastsd	24(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm2
	vfnmadd231pd	%ymm7, %ymm13, %ymm6
	vfnmadd231pd	%ymm11, %ymm13, %ymm10
	vbroadcastsd	24(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm1
	vfnmadd231pd	%ymm7, %ymm13, %ymm5
	vfnmadd231pd	%ymm11, %ymm13, %ymm9
	vbroadcastsd	24(%r10), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm0
	vfnmadd231pd	%ymm7, %ymm13, %ymm4
	vfnmadd231pd	%ymm11, %ymm13, %ymm8

0:
	cmpl			$ 2, %r12d
	jle				0f

	// 3rd column
	vbroadcastsd	16(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm1
	vfnmadd231pd	%ymm6, %ymm13, %ymm5
	vfnmadd231pd	%ymm10, %ymm13, %ymm9
	vbroadcastsd	16(%r10), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm0
	vfnmadd231pd	%ymm6, %ymm13, %ymm4
	vfnmadd231pd	%ymm10, %ymm13, %ymm8

0:
	cmpl			$ 1, %r12d
	jle				0f

	// 2nd column
	vbroadcastsd	8(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm0
	vfnmadd231pd	%ymm5, %ymm13, %ymm4
	vfnmadd231pd	%ymm9, %ymm13, %ymm8

0:

	// 1st column

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rln_one_12x4_vs_lib)
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
// r10  <- E
// r11  <- lde
// r12  <- inv_diag_E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLT_INV_12X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rlt_inv_12x4_lib)
#endif
	
	vbroadcastsd	0(%r12), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm0
	vmulpd			%ymm4, %ymm13, %ymm4
	vmulpd			%ymm8, %ymm13, %ymm8
	vbroadcastsd	8(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm1
	vfnmadd231pd	%ymm4, %ymm13, %ymm5
	vfnmadd231pd	%ymm8, %ymm13, %ymm9
	vbroadcastsd	16(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm2
	vfnmadd231pd	%ymm4, %ymm13, %ymm6
	vfnmadd231pd	%ymm8, %ymm13, %ymm10
	vbroadcastsd	24(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm3
	vfnmadd231pd	%ymm4, %ymm13, %ymm7
	vfnmadd231pd	%ymm8, %ymm13, %ymm11

	vbroadcastsd	8(%r12), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm1
	vmulpd			%ymm5, %ymm13, %ymm5
	vmulpd			%ymm9, %ymm13, %ymm9
	vbroadcastsd	16(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm2
	vfnmadd231pd	%ymm5, %ymm13, %ymm6
	vfnmadd231pd	%ymm9, %ymm13, %ymm10
	vbroadcastsd	24(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm3
	vfnmadd231pd	%ymm5, %ymm13, %ymm7
	vfnmadd231pd	%ymm9, %ymm13, %ymm11

	vbroadcastsd	16(%r12), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm2
	vmulpd			%ymm6, %ymm13, %ymm6
	vmulpd			%ymm10, %ymm13, %ymm10
	vbroadcastsd	24(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm3
	vfnmadd231pd	%ymm6, %ymm13, %ymm7
	vfnmadd231pd	%ymm10, %ymm13, %ymm11

	vbroadcastsd	24(%r12), %ymm13
	vmulpd			%ymm3, %ymm13, %ymm3
	vmulpd			%ymm7, %ymm13, %ymm7
	vmulpd			%ymm11, %ymm13, %ymm11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rlt_inv_12x4_lib)
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
// r10  <- E
// r11  <- lde
// r12  <- inv_diag_E
// r13d <- kn
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLT_INV_12X4_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rlt_inv_12x4_vs_lib)
#endif
	
	vbroadcastsd	0(%r12), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm0
	vmulpd			%ymm4, %ymm13, %ymm4
	vmulpd			%ymm8, %ymm13, %ymm8

	cmpl			$ 2, %r13d
	jl				0f // ret

	vbroadcastsd	8(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm1
	vfnmadd231pd	%ymm4, %ymm13, %ymm5
	vfnmadd231pd	%ymm8, %ymm13, %ymm9
	vbroadcastsd	8(%r12), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm1
	vmulpd			%ymm5, %ymm13, %ymm5
	vmulpd			%ymm9, %ymm13, %ymm9

	cmpl			$ 3, %r13d
	jl				0f // ret

	vbroadcastsd	16(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm2
	vfnmadd231pd	%ymm4, %ymm13, %ymm6
	vfnmadd231pd	%ymm8, %ymm13, %ymm10
	vbroadcastsd	16(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm2
	vfnmadd231pd	%ymm5, %ymm13, %ymm6
	vfnmadd231pd	%ymm9, %ymm13, %ymm10
	vbroadcastsd	16(%r12), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm2
	vmulpd			%ymm6, %ymm13, %ymm6
	vmulpd			%ymm10, %ymm13, %ymm10

	cmpl			$ 4, %r13d
	jl				0f // ret

	vbroadcastsd	24(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm3
	vfnmadd231pd	%ymm4, %ymm13, %ymm7
	vfnmadd231pd	%ymm8, %ymm13, %ymm11
	vbroadcastsd	24(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm3
	vfnmadd231pd	%ymm5, %ymm13, %ymm7
	vfnmadd231pd	%ymm9, %ymm13, %ymm11
	vbroadcastsd	24(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm3
	vfnmadd231pd	%ymm6, %ymm13, %ymm7
	vfnmadd231pd	%ymm10, %ymm13, %ymm11
	vbroadcastsd	24(%r12), %ymm13
	vmulpd			%ymm3, %ymm13, %ymm3
	vmulpd			%ymm7, %ymm13, %ymm7
	vmulpd			%ymm11, %ymm13, %ymm11

0:
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rlt_inv_12x4_vs_lib)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = lower
// tran = transposed
// unit diagonal
//
// input arguments:
// r10  <- E
// r11  <- lde
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLT_ONE_12X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rlt_one_12x4_lib)
#endif
	
	vbroadcastsd	8(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm1
	vfnmadd231pd	%ymm4, %ymm13, %ymm5
	vfnmadd231pd	%ymm8, %ymm13, %ymm9
	vbroadcastsd	16(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm2
	vfnmadd231pd	%ymm4, %ymm13, %ymm6
	vfnmadd231pd	%ymm8, %ymm13, %ymm10
	vbroadcastsd	24(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm3
	vfnmadd231pd	%ymm4, %ymm13, %ymm7
	vfnmadd231pd	%ymm8, %ymm13, %ymm11

	vbroadcastsd	16(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm2
	vfnmadd231pd	%ymm5, %ymm13, %ymm6
	vfnmadd231pd	%ymm9, %ymm13, %ymm10
	vbroadcastsd	24(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm3
	vfnmadd231pd	%ymm5, %ymm13, %ymm7
	vfnmadd231pd	%ymm9, %ymm13, %ymm11

	vbroadcastsd	24(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm3
	vfnmadd231pd	%ymm6, %ymm13, %ymm7
	vfnmadd231pd	%ymm10, %ymm13, %ymm11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rlt_one_12x4_lib)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = lower
// tran = transposed
// unit diagonal
//
// input arguments:
// r10  <- E
// r11  <- lde
// r12d <- kn
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLT_ONE_12X4_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rlt_one_12x4_vs_lib)
#endif
	
	cmpl			$ 2, %r12d
	jl				0f // ret

	vbroadcastsd	8(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm1
	vfnmadd231pd	%ymm4, %ymm13, %ymm5
	vfnmadd231pd	%ymm8, %ymm13, %ymm9

	cmpl			$ 3, %r12d
	jl				0f // ret

	vbroadcastsd	16(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm2
	vfnmadd231pd	%ymm4, %ymm13, %ymm6
	vfnmadd231pd	%ymm8, %ymm13, %ymm10
	vbroadcastsd	16(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm2
	vfnmadd231pd	%ymm5, %ymm13, %ymm6
	vfnmadd231pd	%ymm9, %ymm13, %ymm10

	cmpl			$ 4, %r12d
	jl				0f // ret

	vbroadcastsd	24(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm3
	vfnmadd231pd	%ymm4, %ymm13, %ymm7
	vfnmadd231pd	%ymm8, %ymm13, %ymm11
	vbroadcastsd	24(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm3
	vfnmadd231pd	%ymm5, %ymm13, %ymm7
	vfnmadd231pd	%ymm9, %ymm13, %ymm11
	vbroadcastsd	24(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm3
	vfnmadd231pd	%ymm6, %ymm13, %ymm7
	vfnmadd231pd	%ymm10, %ymm13, %ymm11

0:
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rlt_one_12x4_vs_lib)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = upper
// tran = transposed
// requires explicit inverse of diagonal
//
// input arguments:
// r10  <- E
// r11  <- lde
// r12  <- inv_diag_E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RUN_INV_12X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_run_inv_12x4_lib)
#endif
	
	addq	%r11, %r10

	vbroadcastsd	0(%r12), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm0
	vmulpd			%ymm4, %ymm13, %ymm4
	vmulpd			%ymm8, %ymm13, %ymm8
	vbroadcastsd	0(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm1
	vfnmadd231pd	%ymm4, %ymm13, %ymm5
	vfnmadd231pd	%ymm8, %ymm13, %ymm9
	vbroadcastsd	0(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm2
	vfnmadd231pd	%ymm4, %ymm13, %ymm6
	vfnmadd231pd	%ymm8, %ymm13, %ymm10
	vbroadcastsd	0(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm3
	vfnmadd231pd	%ymm4, %ymm13, %ymm7
	vfnmadd231pd	%ymm8, %ymm13, %ymm11

	vbroadcastsd	8(%r12), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm1
	vmulpd			%ymm5, %ymm13, %ymm5
	vmulpd			%ymm9, %ymm13, %ymm9
	vbroadcastsd	8(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm2
	vfnmadd231pd	%ymm5, %ymm13, %ymm6
	vfnmadd231pd	%ymm9, %ymm13, %ymm10
	vbroadcastsd	8(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm3
	vfnmadd231pd	%ymm5, %ymm13, %ymm7
	vfnmadd231pd	%ymm9, %ymm13, %ymm11

	vbroadcastsd	16(%r12), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm2
	vmulpd			%ymm6, %ymm13, %ymm6
	vmulpd			%ymm10, %ymm13, %ymm10
	vbroadcastsd	16(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm3
	vfnmadd231pd	%ymm6, %ymm13, %ymm7
	vfnmadd231pd	%ymm10, %ymm13, %ymm11

	vbroadcastsd	24(%r12), %ymm13
	vmulpd			%ymm3, %ymm13, %ymm3
	vmulpd			%ymm7, %ymm13, %ymm7
	vmulpd			%ymm11, %ymm13, %ymm11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_run_inv_12x4_lib)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = upper
// tran = transposed
// requires explicit inverse of diagonal
//
// input arguments:
// r10  <- E
// r11  <- lde
// r12  <- inv_diag_E
// r13d <- kn
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RUN_INV_12X4_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_run_inv_12x4_vs_lib)
#endif
	
	addq	%r11, %r10

	vbroadcastsd	0(%r12), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm0
	vmulpd			%ymm4, %ymm13, %ymm4
	vmulpd			%ymm8, %ymm13, %ymm8

	cmpl			$ 2, %r13d
	jl				0f // ret

	vbroadcastsd	0(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm1
	vfnmadd231pd	%ymm4, %ymm13, %ymm5
	vfnmadd231pd	%ymm8, %ymm13, %ymm9
	vbroadcastsd	8(%r12), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm1
	vmulpd			%ymm5, %ymm13, %ymm5
	vmulpd			%ymm9, %ymm13, %ymm9

	cmpl			$ 3, %r13d
	jl				0f // ret

	vbroadcastsd	0(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm2
	vfnmadd231pd	%ymm4, %ymm13, %ymm6
	vfnmadd231pd	%ymm8, %ymm13, %ymm10
	vbroadcastsd	8(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm2
	vfnmadd231pd	%ymm5, %ymm13, %ymm6
	vfnmadd231pd	%ymm9, %ymm13, %ymm10
	vbroadcastsd	16(%r12), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm2
	vmulpd			%ymm6, %ymm13, %ymm6
	vmulpd			%ymm10, %ymm13, %ymm10

	cmpl			$ 4, %r13d
	jl				0f // ret

	vbroadcastsd	0(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm3
	vfnmadd231pd	%ymm4, %ymm13, %ymm7
	vfnmadd231pd	%ymm8, %ymm13, %ymm11
	vbroadcastsd	8(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm3
	vfnmadd231pd	%ymm5, %ymm13, %ymm7
	vfnmadd231pd	%ymm9, %ymm13, %ymm11
	vbroadcastsd	16(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm3
	vfnmadd231pd	%ymm6, %ymm13, %ymm7
	vfnmadd231pd	%ymm10, %ymm13, %ymm11
	vbroadcastsd	24(%r12), %ymm13
	vmulpd			%ymm3, %ymm13, %ymm3
	vmulpd			%ymm7, %ymm13, %ymm7
	vmulpd			%ymm11, %ymm13, %ymm11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_run_inv_12x4_vs_lib)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = upper
// tran = transposed
// unit diagonal
//
// input arguments:
// r10  <- E
// r11  <- lde
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RUN_ONE_12X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_run_one_12x4_lib)
#endif
	
	addq	%r11, %r10

	vbroadcastsd	0(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm1
	vfnmadd231pd	%ymm4, %ymm13, %ymm5
	vfnmadd231pd	%ymm8, %ymm13, %ymm9
	vbroadcastsd	0(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm2
	vfnmadd231pd	%ymm4, %ymm13, %ymm6
	vfnmadd231pd	%ymm8, %ymm13, %ymm10
	vbroadcastsd	0(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm3
	vfnmadd231pd	%ymm4, %ymm13, %ymm7
	vfnmadd231pd	%ymm8, %ymm13, %ymm11

	vbroadcastsd	8(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm2
	vfnmadd231pd	%ymm5, %ymm13, %ymm6
	vfnmadd231pd	%ymm9, %ymm13, %ymm10
	vbroadcastsd	8(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm3
	vfnmadd231pd	%ymm5, %ymm13, %ymm7
	vfnmadd231pd	%ymm9, %ymm13, %ymm11

	vbroadcastsd	16(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm3
	vfnmadd231pd	%ymm6, %ymm13, %ymm7
	vfnmadd231pd	%ymm10, %ymm13, %ymm11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_run_one_12x4_lib)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = upper
// tran = transposed
// unit diagonal
//
// input arguments:
// r10  <- E
// r11  <- lde
// r12d <- kn
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RUN_ONE_12X4_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_run_one_12x4_vs_lib)
#endif
	
	addq	%r11, %r10

	cmpl			$ 2, %r12d
	jl				0f // ret

	vbroadcastsd	0(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm1
	vfnmadd231pd	%ymm4, %ymm13, %ymm5
	vfnmadd231pd	%ymm8, %ymm13, %ymm9

	cmpl			$ 3, %r12d
	jl				0f // ret

	vbroadcastsd	0(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm2
	vfnmadd231pd	%ymm4, %ymm13, %ymm6
	vfnmadd231pd	%ymm8, %ymm13, %ymm10
	vbroadcastsd	8(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm2
	vfnmadd231pd	%ymm5, %ymm13, %ymm6
	vfnmadd231pd	%ymm9, %ymm13, %ymm10

	cmpl			$ 4, %r12d
	jl				0f // ret

	vbroadcastsd	0(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm3
	vfnmadd231pd	%ymm4, %ymm13, %ymm7
	vfnmadd231pd	%ymm8, %ymm13, %ymm11
	vbroadcastsd	8(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm3
	vfnmadd231pd	%ymm5, %ymm13, %ymm7
	vfnmadd231pd	%ymm9, %ymm13, %ymm11
	vbroadcastsd	16(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm3
	vfnmadd231pd	%ymm6, %ymm13, %ymm7
	vfnmadd231pd	%ymm10, %ymm13, %ymm11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_run_one_12x4_vs_lib)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = upper
// tran = transposed
// requires explicit inverse of diagonal
//
// input arguments:
// r10  <- E
// r11  <- lde
// r12  <- inv_diag_E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RUT_INV_12X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rut_inv_12x4_lib)
#endif
	
	addq	%r11, %r10

	// 4th column
	vbroadcastsd	24(%r12), %ymm13
	vmulpd			%ymm3, %ymm13, %ymm3
	vmulpd			%ymm7, %ymm13, %ymm7
	vmulpd			%ymm11, %ymm13, %ymm11
	vbroadcastsd	16(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm2
	vfnmadd231pd	%ymm7, %ymm13, %ymm6
	vfnmadd231pd	%ymm11, %ymm13, %ymm10
	vbroadcastsd	8(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm1
	vfnmadd231pd	%ymm7, %ymm13, %ymm5
	vfnmadd231pd	%ymm11, %ymm13, %ymm9
	vbroadcastsd	0(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm0
	vfnmadd231pd	%ymm7, %ymm13, %ymm4
	vfnmadd231pd	%ymm11, %ymm13, %ymm8

	// 3rd column
	vbroadcastsd	16(%r12), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm2
	vmulpd			%ymm6, %ymm13, %ymm6
	vmulpd			%ymm10, %ymm13, %ymm10
	vbroadcastsd	8(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm1
	vfnmadd231pd	%ymm6, %ymm13, %ymm5
	vfnmadd231pd	%ymm10, %ymm13, %ymm9
	vbroadcastsd	0(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm0
	vfnmadd231pd	%ymm6, %ymm13, %ymm4
	vfnmadd231pd	%ymm10, %ymm13, %ymm8

	// 2nd column
	vbroadcastsd	8(%r12), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm1
	vmulpd			%ymm5, %ymm13, %ymm5
	vmulpd			%ymm9, %ymm13, %ymm9
	vbroadcastsd	0(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm0
	vfnmadd231pd	%ymm5, %ymm13, %ymm4
	vfnmadd231pd	%ymm9, %ymm13, %ymm8

	// 1st column
	vbroadcastsd	0(%r12), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm0
	vmulpd			%ymm4, %ymm13, %ymm4
	vmulpd			%ymm8, %ymm13, %ymm8

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rut_inv_12x4_lib)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = upper
// tran = transposed
// requires explicit inverse of diagonal
//
// input arguments:
// r10  <- E
// r11  <- lde
// r12  <- inv_diag_E
// r13  <- n1
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RUT_INV_12X4_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rut_inv_12x4_vs_lib)
#endif
	
	addq	%r11, %r10

	cmpl			$ 3, %r13d
	jle				0f

	// 4th column
	vbroadcastsd	24(%r12), %ymm13
	vmulpd			%ymm3, %ymm13, %ymm3
	vmulpd			%ymm7, %ymm13, %ymm7
	vmulpd			%ymm11, %ymm13, %ymm11
	vbroadcastsd	16(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm2
	vfnmadd231pd	%ymm7, %ymm13, %ymm6
	vfnmadd231pd	%ymm11, %ymm13, %ymm10
	vbroadcastsd	8(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm1
	vfnmadd231pd	%ymm7, %ymm13, %ymm5
	vfnmadd231pd	%ymm11, %ymm13, %ymm9
	vbroadcastsd	0(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm0
	vfnmadd231pd	%ymm7, %ymm13, %ymm4
	vfnmadd231pd	%ymm11, %ymm13, %ymm8

0:
	cmpl			$ 2, %r13d
	jle				0f

	// 3rd column
	vbroadcastsd	16(%r12), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm2
	vmulpd			%ymm6, %ymm13, %ymm6
	vmulpd			%ymm10, %ymm13, %ymm10
	vbroadcastsd	8(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm1
	vfnmadd231pd	%ymm6, %ymm13, %ymm5
	vfnmadd231pd	%ymm10, %ymm13, %ymm9
	vbroadcastsd	0(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm0
	vfnmadd231pd	%ymm6, %ymm13, %ymm4
	vfnmadd231pd	%ymm10, %ymm13, %ymm8

0:
	cmpl			$ 1, %r13d
	jle				0f

	// 2nd column
	vbroadcastsd	8(%r12), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm1
	vmulpd			%ymm5, %ymm13, %ymm5
	vmulpd			%ymm9, %ymm13, %ymm9
	vbroadcastsd	0(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm0
	vfnmadd231pd	%ymm5, %ymm13, %ymm4
	vfnmadd231pd	%ymm9, %ymm13, %ymm8

0:

	// 1st column
	vbroadcastsd	0(%r12), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm0
	vmulpd			%ymm4, %ymm13, %ymm4
	vmulpd			%ymm8, %ymm13, %ymm8

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rut_inv_12x4_vs_lib)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = upper
// tran = transposed
// unit diagonal
//
// input arguments:
// r10  <- E
// r11  <- lde
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RUT_ONE_12X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rut_one_12x4_lib)
#endif
	
	addq	%r11, %r10

	// 4th column
	vbroadcastsd	16(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm2
	vfnmadd231pd	%ymm7, %ymm13, %ymm6
	vfnmadd231pd	%ymm11, %ymm13, %ymm10
	vbroadcastsd	8(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm1
	vfnmadd231pd	%ymm7, %ymm13, %ymm5
	vfnmadd231pd	%ymm11, %ymm13, %ymm9
	vbroadcastsd	0(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm0
	vfnmadd231pd	%ymm7, %ymm13, %ymm4
	vfnmadd231pd	%ymm11, %ymm13, %ymm8

	// 3rd column
	vbroadcastsd	8(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm1
	vfnmadd231pd	%ymm6, %ymm13, %ymm5
	vfnmadd231pd	%ymm10, %ymm13, %ymm9
	vbroadcastsd	0(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm0
	vfnmadd231pd	%ymm6, %ymm13, %ymm4
	vfnmadd231pd	%ymm10, %ymm13, %ymm8

	// 2nd column
	vbroadcastsd	0(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm0
	vfnmadd231pd	%ymm5, %ymm13, %ymm4
	vfnmadd231pd	%ymm9, %ymm13, %ymm8

	// 1st column

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rut_one_12x4_lib)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = upper
// tran = transposed
// unit diagonal
//
// input arguments:
// r10  <- E
// r11  <- lde
// r12  <- n1
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RUT_ONE_12X4_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rut_one_12x4_vs_lib)
#endif
	
	addq	%r11, %r10

	cmpl			$ 3, %r12d
	jle				0f

	// 4th column
	vbroadcastsd	16(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm2
	vfnmadd231pd	%ymm7, %ymm13, %ymm6
	vfnmadd231pd	%ymm11, %ymm13, %ymm10
	vbroadcastsd	8(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm1
	vfnmadd231pd	%ymm7, %ymm13, %ymm5
	vfnmadd231pd	%ymm11, %ymm13, %ymm9
	vbroadcastsd	0(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm0
	vfnmadd231pd	%ymm7, %ymm13, %ymm4
	vfnmadd231pd	%ymm11, %ymm13, %ymm8

0:
	cmpl			$ 2, %r12d
	jle				0f

	// 3rd column
	vbroadcastsd	8(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm1
	vfnmadd231pd	%ymm6, %ymm13, %ymm5
	vfnmadd231pd	%ymm10, %ymm13, %ymm9
	vbroadcastsd	0(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm0
	vfnmadd231pd	%ymm6, %ymm13, %ymm4
	vfnmadd231pd	%ymm10, %ymm13, %ymm8

0:
	cmpl			$ 1, %r12d
	jle				0f

	// 2nd column
	vbroadcastsd	0(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm0
	vfnmadd231pd	%ymm5, %ymm13, %ymm4
	vfnmadd231pd	%ymm9, %ymm13, %ymm8

0:

	// 1st column

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rut_one_12x4_vs_lib)
#endif





// common inner routine with file scope
//
// LU factorization without pivoting
// middle kernel
//
// input arguments:
// r10  <- E
// r11  <- sde
// r12  <- inv_diag_D
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d51 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
// ymm8 <- [d80 d90 da0 db0]
// ymm9 <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- E
// r11  <- sde
// r12  <- inv_diag_D
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d51 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
// ymm8 <- [d80 d90 da0 db0]
// ymm9 <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DGETRF_M_12X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_edge_dgetrf_m_12x4_lib)
#endif
	
	// compute E
	movq	%r11, %r13
	negq	%r13
	leaq	0(%r10, %r13, 4), %r10
	leaq	0(%r11, %r11, 2), %r14

	// solve upper 4x4 & correct lower 8x4

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovupd			0(%r10), %ymm12
	vblendpd		$ 0x1, %ymm15, %ymm12, %ymm12
	vmovupd			32(%r10), %ymm14
	vmovupd			64(%r10), %ymm15
	vpermpd			$ 0x00, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0x00, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0x00, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0x00, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovupd			0(%r10, %r11), %ymm12
	vblendpd		$ 0x3, %ymm15, %ymm12, %ymm12
	vmovupd			32(%r10, %r11), %ymm14
	vmovupd			64(%r10, %r11), %ymm15
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0x55, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0x55, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0x55, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovupd			0(%r10, %r11, 2), %ymm12
	vblendpd		$ 0x7, %ymm15, %ymm12, %ymm12
	vmovupd			32(%r10, %r11, 2), %ymm14
	vmovupd			64(%r10, %r11, 2), %ymm15
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0xaa, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0xaa, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0xaa, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11

	vmovupd			32(%r10, %r14), %ymm14
	vmovupd			64(%r10, %r14), %ymm15
	vpermpd			$ 0xff, %ymm0, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0xff, %ymm1, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0xff, %ymm2, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0xff, %ymm3, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11


	// factorize lower 8x4

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC05(%rip), %xmm14 // 1.0
#elif defined(OS_MAC)
	vmovsd			LC05(%rip), %xmm14 // 1.0
#endif
//	vmovddup		%xmm14, %xmm14

	// first column
//	vblendpd		$ 0x1, %ymm4, %ymm12, %ymm12
	vmovapd			%ymm4, %ymm12
	vdivsd			%xmm4, %xmm14, %xmm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovsd			%xmm13, 0(%r12)
	vmulpd			%ymm4, %ymm13, %ymm4
	vmulpd			%ymm8, %ymm13, %ymm8
	vblendpd		$ 0x1, %ymm12, %ymm4, %ymm4

	// second column
	vpermpd			$ 0x00, %ymm5, %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm5
	vfnmadd231pd	%ymm8, %ymm13, %ymm9
	vblendpd		$ 0x2, %ymm5, %ymm13, %ymm12

	vpermilpd		$ 0x3, %xmm5, %xmm13
	vdivsd			%xmm13, %xmm14, %xmm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovsd			%xmm13, 8(%r12)
	vmulpd			%ymm5, %ymm13, %ymm5
	vmulpd			%ymm9, %ymm13, %ymm9
	vblendpd		$ 0x3, %ymm12, %ymm5, %ymm5

	// third column
	vpermpd			$ 0x00, %ymm6, %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm6
	vfnmadd231pd	%ymm8, %ymm13, %ymm10
	vblendpd		$ 0x2, %ymm6, %ymm13, %ymm12

	vpermpd			$ 0x55, %ymm6, %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm6
	vfnmadd231pd	%ymm9, %ymm13, %ymm10
	vblendpd		$ 0x4, %ymm6, %ymm12, %ymm12

	vpermpd			$ 0xaa, %ymm6, %ymm13
	vdivsd			%xmm13, %xmm14, %xmm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovsd			%xmm13, 16(%r12)
	vmulpd			%ymm6, %ymm13, %ymm6
	vmulpd			%ymm10, %ymm13, %ymm10
	vblendpd		$ 0x7, %ymm12, %ymm6, %ymm6

	// fourth column
	vpermpd			$ 0x00, %ymm7, %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm7
	vfnmadd231pd	%ymm8, %ymm13, %ymm11
	vblendpd		$ 0x2, %ymm7, %ymm13, %ymm12

	vpermpd			$ 0x55, %ymm7, %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm7
	vfnmadd231pd	%ymm9, %ymm13, %ymm11
	vblendpd		$ 0x4, %ymm7, %ymm12, %ymm12

	vpermpd			$ 0xaa, %ymm7, %ymm13
	vfnmadd231pd	%ymm6, %ymm13, %ymm7
	vfnmadd231pd	%ymm10, %ymm13, %ymm11
	vblendpd		$ 0x8, %ymm7, %ymm12, %ymm12
	
	vpermpd			$ 0xff, %ymm7, %ymm13
	vdivsd			%xmm13, %xmm14, %xmm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovsd			%xmm13, 24(%r12)
//	vmulpd			%ymm7, %ymm13, %ymm7
	vmulpd			%ymm11, %ymm13, %ymm11
	vblendpd		$ 0x7, %ymm12, %ymm7, %ymm7

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dgetrf_m_12x4_lib)
#endif





// common inner routine with file scope
//
// LU factorization without pivoting
// right kernel
//
// input arguments:
// r10  <- E
// r11  <- sde
// r12  <- inv_diag_D
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d51 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
// ymm8 <- [d80 d90 da0 db0]
// ymm9 <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- E
// r11  <- sde
// r12  <- inv_diag_D
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d51 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
// ymm8 <- [d80 d90 da0 db0]
// ymm9 <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DGETRF_R_12X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_edge_dgetrf_r_12x4_lib)
#endif
	
	// compute E
	movq	%r11, %r13
	negq	%r13
	leaq	0(%r10, %r13, 8), %r10
	leaq	0(%r11, %r11, 2), %r14

	// solve upper 8x4 & correct lower 4x4

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovupd			0(%r10), %ymm12
	vblendpd		$ 0x1, %ymm15, %ymm12, %ymm12
	vmovupd			32(%r10), %ymm14
	vmovupd			64(%r10), %ymm15
	vpermpd			$ 0x00, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0x00, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0x00, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0x00, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovupd			0(%r10, %r11), %ymm12
	vblendpd		$ 0x3, %ymm15, %ymm12, %ymm12
	vmovupd			32(%r10, %r11), %ymm14
	vmovupd			64(%r10, %r11), %ymm15
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0x55, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0x55, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0x55, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovupd			0(%r10, %r11, 2), %ymm12
	vblendpd		$ 0x7, %ymm15, %ymm12, %ymm12
	vmovupd			32(%r10, %r11, 2), %ymm14
	vmovupd			64(%r10, %r11, 2), %ymm15
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0xaa, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0xaa, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0xaa, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11

	vmovupd			32(%r10, %r14), %ymm14
	vmovupd			64(%r10, %r14), %ymm15
	vpermpd			$ 0xff, %ymm0, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0xff, %ymm1, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0xff, %ymm2, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0xff, %ymm3, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11


	leaq	0(%r10, %r11, 4), %r10


	vxorpd			%ymm14, %ymm14, %ymm14
	vmovupd			32(%r10), %ymm12
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm12
	vmovupd			64(%r10), %ymm14
	vpermpd			$ 0x00, %ymm4, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm4
	vfnmadd231pd	%ymm14, %ymm13, %ymm8
	vpermpd			$ 0x00, %ymm5, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm5
	vfnmadd231pd	%ymm14, %ymm13, %ymm9
	vpermpd			$ 0x00, %ymm6, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm6
	vfnmadd231pd	%ymm14, %ymm13, %ymm10
	vpermpd			$ 0x00, %ymm7, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm7
	vfnmadd231pd	%ymm14, %ymm13, %ymm11

	vxorpd			%ymm14, %ymm14, %ymm14
	vmovupd			32(%r10, %r11), %ymm12
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm12
	vmovupd			64(%r10, %r11), %ymm14
	vpermpd			$ 0x55, %ymm4, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm4
	vfnmadd231pd	%ymm14, %ymm13, %ymm8
	vpermpd			$ 0x55, %ymm5, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm5
	vfnmadd231pd	%ymm14, %ymm13, %ymm9
	vpermpd			$ 0x55, %ymm6, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm6
	vfnmadd231pd	%ymm14, %ymm13, %ymm10
	vpermpd			$ 0x55, %ymm7, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm7
	vfnmadd231pd	%ymm14, %ymm13, %ymm11

	vxorpd			%ymm14, %ymm14, %ymm14
	vmovupd			32(%r10, %r11, 2), %ymm12
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm12
	vmovupd			64(%r10, %r11, 2), %ymm14
	vpermpd			$ 0xaa, %ymm4, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm4
	vfnmadd231pd	%ymm14, %ymm13, %ymm8
	vpermpd			$ 0xaa, %ymm5, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm5
	vfnmadd231pd	%ymm14, %ymm13, %ymm9
	vpermpd			$ 0xaa, %ymm6, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm6
	vfnmadd231pd	%ymm14, %ymm13, %ymm10
	vpermpd			$ 0xaa, %ymm7, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm7
	vfnmadd231pd	%ymm14, %ymm13, %ymm11

	vmovupd			64(%r10, %r14), %ymm14
	vpermpd			$ 0xff, %ymm4, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm8
	vpermpd			$ 0xff, %ymm5, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm9
	vpermpd			$ 0xff, %ymm6, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm10
	vpermpd			$ 0xff, %ymm7, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm11



	// factorize lower 8x4

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC05(%rip), %xmm14 // 1.0
#elif defined(OS_MAC)
	vmovsd			LC05(%rip), %xmm14 // 1.0
#endif
//	vmovddup		%xmm14, %xmm14

	// first column
//	vblendpd		$ 0x1, %ymm8, %ymm12, %ymm12
	vmovapd			%ymm8, %ymm12
	vdivsd			%xmm8, %xmm14, %xmm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovsd			%xmm13, 0(%r12)
	vmulpd			%ymm8, %ymm13, %ymm8
	vblendpd		$ 0x1, %ymm12, %ymm8, %ymm8

	// second column
	vpermpd			$ 0x00, %ymm9, %ymm13
	vfnmadd231pd	%ymm8, %ymm13, %ymm9
	vblendpd		$ 0x2, %ymm9, %ymm13, %ymm12

	vpermilpd		$ 0x3, %xmm9, %xmm13
	vdivsd			%xmm13, %xmm14, %xmm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovsd			%xmm13, 8(%r12)
	vmulpd			%ymm9, %ymm13, %ymm9
	vblendpd		$ 0x3, %ymm12, %ymm9, %ymm9

	// third column
	vpermpd			$ 0x00, %ymm10, %ymm13
	vfnmadd231pd	%ymm8, %ymm13, %ymm10
	vblendpd		$ 0x2, %ymm10, %ymm13, %ymm12

	vpermpd			$ 0x55, %ymm10, %ymm13
	vfnmadd231pd	%ymm9, %ymm13, %ymm10
	vblendpd		$ 0x4, %ymm10, %ymm12, %ymm12

	vpermpd			$ 0xaa, %ymm10, %ymm13
	vdivsd			%xmm13, %xmm14, %xmm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovsd			%xmm13, 16(%r12)
	vmulpd			%ymm10, %ymm13, %ymm10
	vblendpd		$ 0x7, %ymm12, %ymm10, %ymm10

	// fourth column
	vpermpd			$ 0x00, %ymm11, %ymm13
	vfnmadd231pd	%ymm8, %ymm13, %ymm11
	vblendpd		$ 0x2, %ymm11, %ymm13, %ymm12

	vpermpd			$ 0x55, %ymm11, %ymm13
	vfnmadd231pd	%ymm9, %ymm13, %ymm11
	vblendpd		$ 0x4, %ymm11, %ymm12, %ymm12

	vpermpd			$ 0xaa, %ymm11, %ymm13
	vfnmadd231pd	%ymm10, %ymm13, %ymm11
	vblendpd		$ 0x8, %ymm11, %ymm12, %ymm12
	
	vpermpd			$ 0xff, %ymm11, %ymm13
	vdivsd			%xmm13, %xmm14, %xmm13
//	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovsd			%xmm13, 24(%r12)
//	vmulpd			%ymm11, %ymm13, %ymm11
	vblendpd		$ 0x7, %ymm12, %ymm11, %ymm11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dgetrf_r_12x4_lib)
#endif





// common inner routine with file scope
//
// tran
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- ldc
// ymm0  <-
// ymm1  <-
// ymm2  <-
// ymm3  <-
// ymm4  <-
// ymm5  <-
// ymm6  <-
// ymm7  <-
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_TRAN_12X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_tran_12x4_lib)
#endif

	vunpcklpd	%ymm1, %ymm0, %ymm12
	vunpckhpd	%ymm1, %ymm0, %ymm13
	vunpcklpd	%ymm3, %ymm2, %ymm14
	vunpckhpd	%ymm3, %ymm2, %ymm15

	vperm2f128	$ 0x20, %ymm14, %ymm12, %ymm0
	vperm2f128	$ 0x31, %ymm14, %ymm12, %ymm2
	vperm2f128	$ 0x20, %ymm15, %ymm13, %ymm1
	vperm2f128	$ 0x31, %ymm15, %ymm13, %ymm3

	vunpcklpd	%ymm5, %ymm4, %ymm12
	vunpckhpd	%ymm5, %ymm4, %ymm13
	vunpcklpd	%ymm7, %ymm6, %ymm14
	vunpckhpd	%ymm7, %ymm6, %ymm15

	vperm2f128	$ 0x20, %ymm14, %ymm12, %ymm4
	vperm2f128	$ 0x31, %ymm14, %ymm12, %ymm6
	vperm2f128	$ 0x20, %ymm15, %ymm13, %ymm5
	vperm2f128	$ 0x31, %ymm15, %ymm13, %ymm7

	vunpcklpd	%ymm9, %ymm8, %ymm12
	vunpckhpd	%ymm9, %ymm8, %ymm13
	vunpcklpd	%ymm11, %ymm10, %ymm14
	vunpckhpd	%ymm11, %ymm10, %ymm15

	vperm2f128	$ 0x20, %ymm14, %ymm12, %ymm8
	vperm2f128	$ 0x31, %ymm14, %ymm12, %ymm10
	vperm2f128	$ 0x20, %ymm15, %ymm13, %ymm9
	vperm2f128	$ 0x31, %ymm15, %ymm13, %ymm11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_tran_12x4_lib)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- ldc
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

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_12X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_12x4_lib)
#endif

	// alpha
	vbroadcastsd	0(%r10), %ymm15

	vmulpd		%ymm0, %ymm15, %ymm0
	vmulpd		%ymm1, %ymm15, %ymm1
	vmulpd		%ymm2, %ymm15, %ymm2
	vmulpd		%ymm3, %ymm15, %ymm3
	vmulpd		%ymm4, %ymm15, %ymm4
	vmulpd		%ymm5, %ymm15, %ymm5
	vmulpd		%ymm6, %ymm15, %ymm6
	vmulpd		%ymm7, %ymm15, %ymm7
	vmulpd		%ymm8, %ymm15, %ymm8
	vmulpd		%ymm9, %ymm15, %ymm9
	vmulpd		%ymm10, %ymm15, %ymm10
	vmulpd		%ymm11, %ymm15, %ymm11

	// beta
	vbroadcastsd	0(%r11), %ymm14

	vxorpd		%ymm15, %ymm15, %ymm15 // 0.0

	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm0
	vmovupd		32(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm4
	vmovupd		64(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm8
	addq		%r13, %r12
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm1
	vmovupd		32(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm5
	vmovupd		64(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm9
	addq		%r13, %r12
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm2
	vmovupd		32(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm6
	vmovupd		64(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm10
	addq		%r13, %r12
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm3
	vmovupd		32(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm7
	vmovupd		64(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm11
//	addq		%r13, %r12

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_12x4_lib)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- ldc
// r14d   <- km
// r15d   <- kn
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

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_12X4_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_12x4_vs_lib)
#endif

	// alpha
	vbroadcastsd	0(%r10), %ymm15

	vmulpd		%ymm0, %ymm15, %ymm0
	vmulpd		%ymm1, %ymm15, %ymm1
	vmulpd		%ymm2, %ymm15, %ymm2
	vmulpd		%ymm3, %ymm15, %ymm3
	vmulpd		%ymm4, %ymm15, %ymm4
	vmulpd		%ymm5, %ymm15, %ymm5
	vmulpd		%ymm6, %ymm15, %ymm6
	vmulpd		%ymm7, %ymm15, %ymm7
	vmulpd		%ymm8, %ymm15, %ymm8
	vmulpd		%ymm9, %ymm15, %ymm9
	vmulpd		%ymm10, %ymm15, %ymm10
	vmulpd		%ymm11, %ymm15, %ymm11

	// beta
	vbroadcastsd	0(%r11), %ymm14

	vxorpd		%ymm15, %ymm15, %ymm15 // 0.0
	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end


	vcvtsi2sd	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC04(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC04(%rip), %ymm13
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm13, %ymm13


	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm0
	vmovupd		32(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm4
	vmaskmovpd	64(%r12), %ymm13, %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm8
	addq		%r13, %r12
	cmpl		$ 2, %r15d
	jl			0f // end
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm1
	vmovupd		32(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm5
	vmaskmovpd	64(%r12), %ymm13, %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm9
	addq		%r13, %r12
	cmpl		$ 3, %r15d
	jl			0f // end
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm2
	vmovupd		32(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm6
	vmaskmovpd	64(%r12), %ymm13, %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm10
	addq		%r13, %r12
	cmpl		$ 3, %r15d
	je			0f // end
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm3
	vmovupd		32(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm7
	vmaskmovpd	64(%r12), %ymm13, %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm11
//	addq		%r13, %r12

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_12x4_vs_lib)
#endif





// common inner routine with file scope
//
// scale for alpha=-1 and generic beta
//
// input arguments:
// r10   <- beta
// r11   <- C
// r12   <- ldc
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

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M1B_12X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_scale_m1b_12x4_lib)
#endif

	// beta
	vbroadcastsd	0(%r10), %ymm14

	vxorpd		%ymm15, %ymm15, %ymm15 // 0.0

	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	vmovupd		0(%r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm0
	vmovupd		32(%r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm4
	vmovupd		64(%r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm8
	addq		%r12, %r11
	vmovupd		0(%r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm1
	vmovupd		32(%r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm5
	vmovupd		64(%r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm9
	addq		%r12, %r11
	vmovupd		0(%r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm2
	vmovupd		32(%r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm6
	vmovupd		64(%r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm10
	addq		%r12, %r11
	vmovupd		0(%r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm3
	vmovupd		32(%r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm7
	vmovupd		64(%r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm11
//	addq		%r12, %r11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m1b_12x4_lib)
#endif





// common inner routine with file scope
//
// scale for alpha=-1 and generic beta
//
// input arguments:
// r10   <- beta
// r11   <- C
// r12   <- ldc
// r13d   <- km
// r14d   <- kn
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

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M1B_12X4_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_scale_m1b_12x4_vs_lib)
#endif

	// beta
	vbroadcastsd	0(%r10), %ymm14

	vxorpd		%ymm15, %ymm15, %ymm15 // 0.0
	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end


	vcvtsi2sd	%r13d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC04(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC04(%rip), %ymm13
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm13, %ymm13


	vmovupd		0(%r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm0
	vmovupd		32(%r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm4
	vmaskmovpd	64(%r11), %ymm13, %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm8
	addq		%r12, %r11
	cmpl		$ 2, %r14d
	jl			0f // end
	vmovupd		0(%r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm1
	vmovupd		32(%r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm5
	vmaskmovpd	64(%r11), %ymm13, %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm9
	addq		%r12, %r11
	cmpl		$ 3, %r14d
	jl			0f // end
	vmovupd		0(%r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm2
	vmovupd		32(%r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm6
	vmaskmovpd	64(%r11), %ymm13, %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm10
	addq		%r12, %r11
	cmpl		$ 3, %r14d
	je			0f // end
	vmovupd		0(%r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm3
	vmovupd		32(%r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm7
	vmaskmovpd	64(%r11), %ymm13, %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm11
//	addq		%r12, %r11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m1b_12x4_vs_lib)
#endif





// common inner routine with file scope
//
// scale for alpha=-1 and beta=1
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
	.macro INNER_SCALE_M11_12X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_scale_m11_12x4_lib)
#endif

	vmovupd		0(%r10), %ymm15
	vsubpd		%ymm0, %ymm15, %ymm0
	vmovupd		32(%r10), %ymm15
	vsubpd		%ymm4, %ymm15, %ymm4
	vmovupd		64(%r10), %ymm15
	vsubpd		%ymm8, %ymm15, %ymm8
	addq		%r11, %r10
	vmovupd		0(%r10), %ymm15
	vsubpd		%ymm1, %ymm15, %ymm1
	vmovupd		32(%r10), %ymm15
	vsubpd		%ymm5, %ymm15, %ymm5
	vmovupd		64(%r10), %ymm15
	vsubpd		%ymm9, %ymm15, %ymm9
	addq		%r11, %r10
	vmovupd		0(%r10), %ymm15
	vsubpd		%ymm2, %ymm15, %ymm2
	vmovupd		32(%r10), %ymm15
	vsubpd		%ymm6, %ymm15, %ymm6
	vmovupd		64(%r10), %ymm15
	vsubpd		%ymm10, %ymm15, %ymm10
	addq		%r11, %r10
	vmovupd		0(%r10), %ymm15
	vsubpd		%ymm3, %ymm15, %ymm3
	vmovupd		32(%r10), %ymm15
	vsubpd		%ymm7, %ymm15, %ymm7
	vmovupd		64(%r10), %ymm15
	vsubpd		%ymm11, %ymm15, %ymm11
//	addq		%r11, %r10

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m11_12x4_lib)
#endif





// common inner routine with file scope
//
// scale for alpha=-1 and beta=1
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
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M11_12X4_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_scale_m11_12x4_vs_lib)
#endif

	vcvtsi2sd	%r12d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC04(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC04(%rip), %ymm13
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm13, %ymm13


	vmovupd		0(%r10), %ymm15
	vsubpd		%ymm0, %ymm15, %ymm0
	vmovupd		32(%r10), %ymm15
	vsubpd		%ymm4, %ymm15, %ymm4
	vmaskmovpd	64(%r10), %ymm13, %ymm15
	vsubpd		%ymm8, %ymm15, %ymm8
	addq		%r11, %r10
	cmpl		$ 2, %r13d
	jl			0f // end
	vmovupd		0(%r10), %ymm15
	vsubpd		%ymm1, %ymm15, %ymm1
	vmovupd		32(%r10), %ymm15
	vsubpd		%ymm5, %ymm15, %ymm5
	vmaskmovpd	64(%r10), %ymm13, %ymm15
	vsubpd		%ymm9, %ymm15, %ymm9
	addq		%r11, %r10
	cmpl		$ 3, %r13d
	jl			0f // end
	vmovupd		0(%r10), %ymm15
	vsubpd		%ymm2, %ymm15, %ymm2
	vmovupd		32(%r10), %ymm15
	vsubpd		%ymm6, %ymm15, %ymm6
	vmaskmovpd	64(%r10), %ymm13, %ymm15
	vsubpd		%ymm10, %ymm15, %ymm10
	addq		%r11, %r10
	cmpl		$ 3, %r13d
	je			0f // end
	vmovupd		0(%r10), %ymm15
	vsubpd		%ymm3, %ymm15, %ymm3
	vmovupd		32(%r10), %ymm15
	vsubpd		%ymm7, %ymm15, %ymm7
	vmaskmovpd	64(%r10), %ymm13, %ymm15
	vsubpd		%ymm11, %ymm15, %ymm11
//	addq		%r11, %r10

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m11_12x4_vs_lib)
#endif





// common inner routine with file scope
//
// tran scale for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- ldc
// ymm0  <-
// ymm1  <-
// ymm2  <-
// ymm3  <-
// ymm4  <-
// ymm5  <-
// ymm6  <-
// ymm7  <-
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_4X12_LIB
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_4x12_lib)
#endif

	// alpha
	vbroadcastsd	0(%r10), %ymm15

	vmulpd		%ymm0, %ymm15, %ymm0
	vmulpd		%ymm1, %ymm15, %ymm1
	vmulpd		%ymm2, %ymm15, %ymm2
	vmulpd		%ymm3, %ymm15, %ymm3
	vmulpd		%ymm4, %ymm15, %ymm4
	vmulpd		%ymm5, %ymm15, %ymm5
	vmulpd		%ymm6, %ymm15, %ymm6
	vmulpd		%ymm7, %ymm15, %ymm7
	vmulpd		%ymm8, %ymm15, %ymm8
	vmulpd		%ymm9, %ymm15, %ymm9
	vmulpd		%ymm10, %ymm15, %ymm10
	vmulpd		%ymm11, %ymm15, %ymm11

	// beta
	vbroadcastsd	0(%r11), %ymm14

	vxorpd		%ymm15, %ymm15, %ymm15 // 0.0
	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm0
	addq		%r13, %r12
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm1
	addq		%r13, %r12
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm2
	addq		%r13, %r12
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm3
	addq		%r13, %r12
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm4
	addq		%r13, %r12
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm5
	addq		%r13, %r12
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm6
	addq		%r13, %r12
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm7
	addq		%r13, %r12
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm8
	addq		%r13, %r12
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm9
	addq		%r13, %r12
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm10
	addq		%r13, %r12
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_4x12_lib)
#endif





// common inner routine with file scope
//
// tran scale for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- ldc
// r14d   <- km
// r15d   <- kn
// ymm0  <-
// ymm1  <-
// ymm2  <-
// ymm3  <-
// ymm4  <-
// ymm5  <-
// ymm6  <-
// ymm7  <-
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_4X12_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_4x12_vs_lib)
#endif

	// alpha
	vbroadcastsd	0(%r10), %ymm15

	vmulpd		%ymm0, %ymm15, %ymm0
	vmulpd		%ymm1, %ymm15, %ymm1
	vmulpd		%ymm2, %ymm15, %ymm2
	vmulpd		%ymm3, %ymm15, %ymm3
	vmulpd		%ymm4, %ymm15, %ymm4
	vmulpd		%ymm5, %ymm15, %ymm5
	vmulpd		%ymm6, %ymm15, %ymm6
	vmulpd		%ymm7, %ymm15, %ymm7
	vmulpd		%ymm8, %ymm15, %ymm8
	vmulpd		%ymm9, %ymm15, %ymm9
	vmulpd		%ymm10, %ymm15, %ymm10
	vmulpd		%ymm11, %ymm15, %ymm11

	// beta
	vbroadcastsd	0(%r11), %ymm14

	vxorpd		%ymm15, %ymm15, %ymm15 // 0.0
	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end


	vcvtsi2sd	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %ymm13
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm13, %ymm13


	vmaskmovpd	0(%r12), %ymm13, %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm0
	addq		%r13, %r12
	vmaskmovpd	0(%r12), %ymm13, %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm1
	addq		%r13, %r12
	vmaskmovpd	0(%r12), %ymm13, %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm2
	addq		%r13, %r12
	vmaskmovpd	0(%r12), %ymm13, %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm3
	addq		%r13, %r12
	vmaskmovpd	0(%r12), %ymm13, %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm4
	addq		%r13, %r12
	vmaskmovpd	0(%r12), %ymm13, %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm5
	addq		%r13, %r12
	vmaskmovpd	0(%r12), %ymm13, %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm6
	addq		%r13, %r12
	vmaskmovpd	0(%r12), %ymm13, %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm7
	addq		%r13, %r12
	vmaskmovpd	0(%r12), %ymm13, %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm8
	addq		%r13, %r12
	cmpl		$ 10, %r15d
	jl			0f // end
	vmaskmovpd	0(%r12), %ymm13, %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm9
	addq		%r13, %r12
	cmpl		$ 11, %r15d
	jl			0f // end
	vmaskmovpd	0(%r12), %ymm13, %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm10
	addq		%r13, %r12
	cmpl		$ 11, %r15d
	je			0f // end
	vmaskmovpd	0(%r12), %ymm13, %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_4x12_vs_lib)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- ldd
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

#if MACRO_LEVEL>=1
	.macro INNER_STORE_12X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_store_12x4_lib)
#endif

	vmovupd		%ymm0, 0(%r10)
	vmovupd		%ymm4, 32(%r10)
	vmovupd		%ymm8, 64(%r10)
	addq		%r11, %r10
	vmovupd		%ymm1, 0(%r10)
	vmovupd		%ymm5, 32(%r10)
	vmovupd		%ymm9, 64(%r10)
	addq		%r11, %r10
	vmovupd		%ymm2, 0(%r10)
	vmovupd		%ymm6, 32(%r10)
	vmovupd		%ymm10, 64(%r10)
	addq		%r11, %r10
	vmovupd		%ymm3, 0(%r10)
	vmovupd		%ymm7, 32(%r10)
	vmovupd		%ymm11, 64(%r10)
//	addq	%r11, %r10

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_12x4_lib)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- ldd
// ymm0  <-
// ymm1  <-
// ymm2  <-
// ymm3  <-
// ymm4  <-
// ymm5  <-
// ymm6  <-
// ymm7  <-
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X12_LIB
#else
	.p2align 4,,15
	FUN_START(inner_store_4x12_lib)
#endif

	vmovupd		%ymm0, 0(%r10)
	addq		%r11, %r10
	vmovupd		%ymm1, 0(%r10)
	addq		%r11, %r10
	vmovupd		%ymm2, 0(%r10)
	addq		%r11, %r10
	vmovupd		%ymm3, 0(%r10)
	addq		%r11, %r10
	vmovupd		%ymm4, 0(%r10)
	addq		%r11, %r10
	vmovupd		%ymm5, 0(%r10)
	addq		%r11, %r10
	vmovupd		%ymm6, 0(%r10)
	addq		%r11, %r10
	vmovupd		%ymm7, 0(%r10)
	addq		%r11, %r10
	vmovupd		%ymm8, 0(%r10)
	addq		%r11, %r10
	vmovupd		%ymm9, 0(%r10)
	addq		%r11, %r10
	vmovupd		%ymm10, 0(%r10)
	addq		%r11, %r10
	vmovupd		%ymm11, 0(%r10)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4x12_lib)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- ldd
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_12X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_store_l_12x4_lib)
#endif

	vmovupd		%ymm0, 0(%r10)
	vmovupd		%ymm4, 32(%r10)
	vmovupd		%ymm8, 64(%r10)
	addq		%r11, %r10
	vmovupd		0(%r10), %ymm15
	vblendpd	$ 0x1, %ymm15, %ymm1, %ymm1
	vmovupd		%ymm1, 0(%r10)
	vmovupd		%ymm5, 32(%r10)
	vmovupd		%ymm9, 64(%r10)
	addq		%r11, %r10
	vmovupd		0(%r10), %ymm15
	vblendpd	$ 0x3, %ymm15, %ymm2, %ymm2
	vmovupd		%ymm2, 0(%r10)
	vmovupd		%ymm6, 32(%r10)
	vmovupd		%ymm10, 64(%r10)
	addq		%r11, %r10
	vmovupd		0(%r10), %ymm15
	vblendpd	$ 0x7, %ymm15, %ymm3, %ymm3
	vmovupd		%ymm3, 0(%r10)
	vmovupd		%ymm7, 32(%r10)
	vmovupd		%ymm11, 64(%r10)
//	addq	%r11, %r10

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_12x4_lib)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- ldd
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_U_12X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_store_u_12x4_lib)
#endif

	vmovupd		%ymm0, 0(%r10)
	vmovupd		%ymm4, 32(%r10)
	vmovupd		64(%r10), %ymm15
	vblendpd	$ 0x1, %ymm8, %ymm15, %ymm8
	vmovupd		%ymm8, 64(%r10)
	addq		%r11, %r10
	vmovupd		%ymm1, 0(%r10)
	vmovupd		%ymm5, 32(%r10)
	vmovupd		64(%r10), %ymm15
	vblendpd	$ 0x3, %ymm9, %ymm15, %ymm9
	vmovupd		%ymm9, 64(%r10)
	addq		%r11, %r10
	vmovupd		%ymm2, 0(%r10)
	vmovupd		%ymm6, 32(%r10)
	vmovupd		64(%r10), %ymm15
	vblendpd	$ 0x7, %ymm10, %ymm15, %ymm10
	vmovupd		%ymm10, 64(%r10)
	addq		%r11, %r10
	vmovupd		%ymm3, 0(%r10)
	vmovupd		%ymm7, 32(%r10)
	vmovupd		%ymm11, 64(%r10)
//	addq	%r11, %r10

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_u_12x4_lib)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- ldd
// r12d   <- km
// r13d   <- kn
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

#if MACRO_LEVEL>=1
	.macro INNER_STORE_12X4_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_store_12x4_vs_lib)
#endif

	vcvtsi2sd	%r12d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC04(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC04(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15

	vmovupd		%ymm0, 0(%r10)
	vmovupd		%ymm4, 32(%r10)
	vmaskmovpd	%ymm8, %ymm15, 64(%r10)
	addq		%r11, %r10
	cmpl		$ 2, %r13d
	jl			0f // end
	vmovupd		%ymm1, 0(%r10)
	vmovupd		%ymm5, 32(%r10)
	vmaskmovpd	%ymm9, %ymm15, 64(%r10)
	addq		%r11, %r10
	cmpl		$ 3, %r13d
	jl			0f // end
	vmovupd		%ymm2, 0(%r10)
	vmovupd		%ymm6, 32(%r10)
	vmaskmovpd	%ymm10, %ymm15, 64(%r10)
	addq		%r11, %r10
	cmpl		$ 3, %r13d
	je			0f // end
	vmovupd		%ymm3, 0(%r10)
	vmovupd		%ymm7, 32(%r10)
	vmaskmovpd	%ymm11, %ymm15, 64(%r10)
//	addq	%r11, %r10

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_12x4_vs_lib)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- ldd
// ymm0  <-
// ymm1  <-
// ymm2  <-
// ymm3  <-
// ymm4  <-
// ymm5  <-
// ymm6  <-
// ymm7  <-
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X12_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_store_4x12_vs_lib)
#endif

	vcvtsi2sd	%r12d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15

	vmaskmovpd	%ymm0, %ymm15, 0(%r10)
	addq		%r11, %r10
	vmaskmovpd	%ymm1, %ymm15, 0(%r10)
	addq		%r11, %r10
	vmaskmovpd	%ymm2, %ymm15, 0(%r10)
	addq		%r11, %r10
	vmaskmovpd	%ymm3, %ymm15, 0(%r10)
	addq		%r11, %r10
	vmaskmovpd	%ymm4, %ymm15, 0(%r10)
	addq		%r11, %r10
	vmaskmovpd	%ymm5, %ymm15, 0(%r10)
	addq		%r11, %r10
	vmaskmovpd	%ymm6, %ymm15, 0(%r10)
	addq		%r11, %r10
	vmaskmovpd	%ymm7, %ymm15, 0(%r10)
	addq		%r11, %r10
	vmaskmovpd	%ymm8, %ymm15, 0(%r10)
	addq		%r11, %r10
	cmpl		$ 10, %r13d
	jl			0f // end
	vmaskmovpd	%ymm9, %ymm15, 0(%r10)
	addq		%r11, %r10
	cmpl		$ 11, %r13d
	jl			0f // end
	vmaskmovpd	%ymm10, %ymm15, 0(%r10)
	addq		%r11, %r10
	cmpl		$ 11, %r13d
	je			0f // end
	vmaskmovpd	%ymm11, %ymm15, 0(%r10)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4x12_vs_lib)
#endif





// common inner routine with file scope
//
// store n vs
//
// input arguments:
// r10   <- D
// r11  <- ldd
// r12d   <- km
// r13d   <- kn
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d03 d12 d21 d30]
// ymm3  <- [d02 d13 d20 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_12X4_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_store_l_12x4_vs_lib)
#endif
	
	vcvtsi2sd	%r12d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC04(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC04(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm13

	vmovupd		%ymm0, 0(%r10)
	vmovupd		%ymm4, 32(%r10)
	vmaskmovpd	%ymm8, %ymm13, 64(%r10)
	addq		%r11, %r10
	cmpl		$ 2, %r13d
	jl			0f // end
	vmovupd		0(%r10), %ymm15
	vblendpd	$ 0x1, %ymm15, %ymm1, %ymm1
	vmovupd		%ymm1, 0(%r10)
	vmovupd		%ymm5, 32(%r10)
	vmaskmovpd	%ymm9, %ymm13, 64(%r10)
	addq		%r11, %r10
	cmpl		$ 3, %r13d
	jl			0f // end
	vmovupd		0(%r10), %ymm15
	vblendpd	$ 0x3, %ymm15, %ymm2, %ymm2
	vmovupd		%ymm2, 0(%r10)
	vmovupd		%ymm6, 32(%r10)
	vmaskmovpd	%ymm10, %ymm13, 64(%r10)
	addq		%r11, %r10
	cmpl		$ 3, %r13d
	je			0f // end
	vmovupd		0(%r10), %ymm15
	vblendpd	$ 0x7, %ymm15, %ymm3, %ymm3
	vmovupd		%ymm3, 0(%r10)
	vmovupd		%ymm7, 32(%r10)
	vmaskmovpd	%ymm11, %ymm13, 64(%r10)
//	addq	%r11, %r10

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_12x4_vs_lib)
#endif





// common inner routine with file scope
//
// store n vs
//
// input arguments:
// r10   <- D
// r11  <- ldd
// r12d   <- km
// r13d   <- kn
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d03 d12 d21 d30]
// ymm3  <- [d02 d13 d20 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_U_12X4_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_store_u_12x4_vs_lib)
#endif
	
	vcvtsi2sd	%r12d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC04(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC04(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm13

	vmovupd		%ymm0, 0(%r10)
	vmovupd		%ymm4, 32(%r10)
	vmovupd		64(%r10), %ymm15
	vblendpd	$ 0x1, %ymm8, %ymm15, %ymm8
	vmaskmovpd	%ymm8, %ymm13, 64(%r10)
	addq		%r11, %r10
	cmpl		$ 2, %r13d
	jl			0f // end
	vmovupd		%ymm1, 0(%r10)
	vmovupd		%ymm5, 32(%r10)
	vmovupd		64(%r10), %ymm15
	vblendpd	$ 0x3, %ymm9, %ymm15, %ymm9
	vmaskmovpd	%ymm9, %ymm13, 64(%r10)
	addq		%r11, %r10
	cmpl		$ 3, %r13d
	jl			0f // end
	vmovupd		%ymm2, 0(%r10)
	vmovupd		%ymm6, 32(%r10)
	vmovupd		64(%r10), %ymm15
	vblendpd	$ 0x7, %ymm10, %ymm15, %ymm10
	vmaskmovpd	%ymm10, %ymm13, 64(%r10)
	addq		%r11, %r10
	cmpl		$ 3, %r13d
	je			0f // end
	vmovupd		%ymm3, 0(%r10)
	vmovupd		%ymm7, 32(%r10)
	vmaskmovpd	%ymm11, %ymm13, 64(%r10)
//	addq	%r11, %r10

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_u_12x4_vs_lib)
#endif





// common inner routine with file scope
//
// scale store for generic alpha and beta=1
//
// input arguments:
// r10   <- alpha
// r11   <- D
// r12   <- ldd
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

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_A1_STORE_12X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_scale_a1_store_12x4_lib)
#endif

	// alpha
	vbroadcastsd	0(%r10), %ymm15

	vmovupd		0(%r11), %ymm12
	vfmadd231pd	%ymm0, %ymm15, %ymm12
	vmovupd		32(%r11), %ymm13
	vfmadd231pd	%ymm4, %ymm15, %ymm13
	vmovupd		64(%r11), %ymm14
	vfmadd231pd	%ymm8, %ymm15, %ymm14
	vmovupd		%ymm12, 0(%r11)
	vmovupd		%ymm13, 32(%r11)
	vmovupd		%ymm14, 64(%r11)
	addq		%r12, %r11

	vmovupd		0(%r11), %ymm12
	vfmadd231pd	%ymm1, %ymm15, %ymm12
	vmovupd		32(%r11), %ymm13
	vfmadd231pd	%ymm5, %ymm15, %ymm13
	vmovupd		64(%r11), %ymm14
	vfmadd231pd	%ymm9, %ymm15, %ymm14
	vmovupd		%ymm12, 0(%r11)
	vmovupd		%ymm13, 32(%r11)
	vmovupd		%ymm14, 64(%r11)
	addq		%r12, %r11

	vmovupd		0(%r11), %ymm12
	vfmadd231pd	%ymm2, %ymm15, %ymm12
	vmovupd		32(%r11), %ymm13
	vfmadd231pd	%ymm6, %ymm15, %ymm13
	vmovupd		64(%r11), %ymm14
	vfmadd231pd	%ymm10, %ymm15, %ymm14
	vmovupd		%ymm12, 0(%r11)
	vmovupd		%ymm13, 32(%r11)
	vmovupd		%ymm14, 64(%r11)
	addq		%r12, %r11

	vmovupd		0(%r11), %ymm12
	vfmadd231pd	%ymm3, %ymm15, %ymm12
	vmovupd		32(%r11), %ymm13
	vfmadd231pd	%ymm7, %ymm15, %ymm13
	vmovupd		64(%r11), %ymm14
	vfmadd231pd	%ymm11, %ymm15, %ymm14
	vmovupd		%ymm12, 0(%r11)
	vmovupd		%ymm13, 32(%r11)
	vmovupd		%ymm14, 64(%r11)
//	addq		%r12, %r11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_a1_store_12x4_lib)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10   <- alpha
// r11  <- D
// r12  <- ldd
// ymm0  <-
// ymm1  <-
// ymm2  <-
// ymm3  <-
// ymm4  <-
// ymm5  <-
// ymm6  <-
// ymm7  <-
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_A1_STORE_4X12_LIB
#else
	.p2align 4,,15
	FUN_START(inner_scale_a1_store_4x12_lib)
#endif

	// alpha
	vbroadcastsd	0(%r10), %ymm15

	leaq		0(%r12, %r12, 2), %r13

	vmovupd		0(%r11), %ymm12
	vmovupd		0(%r11, %r12), %ymm13
	vmovupd		0(%r11, %r12, 2), %ymm14
	vfmadd231pd	%ymm0, %ymm15, %ymm12
	vfmadd231pd	%ymm1, %ymm15, %ymm13
	vfmadd231pd	%ymm2, %ymm15, %ymm14
	vmovupd		%ymm12, 0(%r11)
	vmovupd		%ymm13, 0(%r11, %r12)
	vmovupd		%ymm14, 0(%r11, %r12, 2)
	addq		%r13, %r11

	vmovupd		0(%r11), %ymm12
	vmovupd		0(%r11, %r12), %ymm13
	vmovupd		0(%r11, %r12, 2), %ymm14
	vfmadd231pd	%ymm3, %ymm15, %ymm12
	vfmadd231pd	%ymm4, %ymm15, %ymm13
	vfmadd231pd	%ymm5, %ymm15, %ymm14
	vmovupd		%ymm12, 0(%r11)
	vmovupd		%ymm13, 0(%r11, %r12)
	vmovupd		%ymm14, 0(%r11, %r12, 2)
	addq		%r13, %r11

	vmovupd		0(%r11), %ymm12
	vmovupd		0(%r11, %r12), %ymm13
	vmovupd		0(%r11, %r12, 2), %ymm14
	vfmadd231pd	%ymm6, %ymm15, %ymm12
	vfmadd231pd	%ymm7, %ymm15, %ymm13
	vfmadd231pd	%ymm8, %ymm15, %ymm14
	vmovupd		%ymm12, 0(%r11)
	vmovupd		%ymm13, 0(%r11, %r12)
	vmovupd		%ymm14, 0(%r11, %r12, 2)
	addq		%r13, %r11

	vmovupd		0(%r11), %ymm12
	vmovupd		0(%r11, %r12), %ymm13
	vmovupd		0(%r11, %r12, 2), %ymm14
	vfmadd231pd	%ymm9, %ymm15, %ymm12
	vfmadd231pd	%ymm10, %ymm15, %ymm13
	vfmadd231pd	%ymm11, %ymm15, %ymm14
	vmovupd		%ymm12, 0(%r11)
	vmovupd		%ymm13, 0(%r11, %r12)
	vmovupd		%ymm14, 0(%r11, %r12, 2)
	addq		%r13, %r11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_a1_store_4x12_lib)
#endif





// common inner routine with file scope
//
// prefetch
//
// input arguments:
// r10  <- D
// r11  <- ldd
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_PREFETCH0_12X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_prefetch0_12x4_lib)
#endif

	leaq		0(%r11, %r11, 2), %r12
	prefetcht0	0(%r10)				// 0
	prefetcht0	64(%r10)			// 0
	prefetcht0	95(%r10)			// 0
	prefetcht0	0(%r10, %r11)	// 1
	prefetcht0	64(%r10, %r11)	// 1
	prefetcht0	95(%r10, %r11)	// 1
	prefetcht0	0(%r10, %r11, 2)	// 2
	prefetcht0	64(%r10, %r11, 2)	// 2
	prefetcht0	95(%r10, %r11, 2)	// 2
	prefetcht0	0(%r10, %r12)	// 3
	prefetcht0	64(%r10, %r12)	// 3
	prefetcht0	95(%r10, %r12)	// 3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_prefetch0_12x4_lib)
#endif





// common inner routine with file scope
//
// prefetch
//
// input arguments:
// r10  <- D
// r11  <- ldd
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_PREFETCH1_12X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_prefetch1_12x4_lib)
#endif

	leaq		0(%r11, %r11, 2), %r12
	prefetcht1	0(%r10)				// 0
	prefetcht1	64(%r10)			// 0
	prefetcht1	95(%r10)			// 0
	prefetcht1	0(%r10, %r11)	// 1
	prefetcht1	64(%r10, %r11)	// 1
	prefetcht1	95(%r10, %r11)	// 1
	prefetcht1	0(%r10, %r11, 2)	// 2
	prefetcht1	64(%r10, %r11, 2)	// 2
	prefetcht1	95(%r10, %r11, 2)	// 2
	prefetcht1	0(%r10, %r12)	// 3
	prefetcht1	64(%r10, %r12)	// 3
	prefetcht1	95(%r10, %r12)	// 3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_prefetch1_12x4_lib)
#endif





// common inner routine with file scope
//
// prefetch
//
// input arguments:
// r10  <- D
// r11  <- ldd
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_PREFETCH0_4X12_LIB
#else
	.p2align 4,,15
	FUN_START(inner_prefetch0_4x12_lib)
#endif

	leaq		0(%r11, %r11, 2), %r12
	leaq		0(%r10, %r11, 4), %r13
	leaq		0(%r10, %r11, 8), %r14
#if 0
	prefetcht0	0(%r10)				// 0
	prefetcht0	0(%r10, %r11)	// 1
	prefetcht0	0(%r10, %r11, 2)	// 2
	prefetcht0	0(%r10, %r12)	// 3
	prefetcht0	0(%r13)				// 4
	prefetcht0	0(%r13, %r11)	// 5
	prefetcht0	0(%r13, %r11, 2)	// 6
	prefetcht0	0(%r13, %r12)	// 7
	prefetcht0	0(%r14)				// 8
	prefetcht0	0(%r14, %r11)	// 9
	prefetcht0	0(%r14, %r11, 2)	// 10
	prefetcht0	0(%r14, %r12)	// 11
#else
	prefetcht0	31(%r10)				// 0
	prefetcht0	31(%r10, %r11)	// 1
	prefetcht0	31(%r10, %r11, 2)	// 2
	prefetcht0	31(%r10, %r12)	// 3
	prefetcht0	31(%r13)				// 4
	prefetcht0	31(%r13, %r11)	// 5
	prefetcht0	31(%r13, %r11, 2)	// 6
	prefetcht0	31(%r13, %r12)	// 7
	prefetcht0	31(%r14)				// 8
	prefetcht0	31(%r14, %r11)	// 9
	prefetcht0	31(%r14, %r11, 2)	// 10
	prefetcht0	31(%r14, %r12)	// 11
#endif

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_prefetch0_4x12_lib)
#endif





// common inner routine with file scope
//
// prefetch
//
// input arguments:
// r10  <- D
// r11  <- ldd
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_PREFETCH1_4X12_LIB
#else
	.p2align 4,,15
	FUN_START(inner_prefetch1_4x12_lib)
#endif

	leaq		0(%r11, %r11, 2), %r12
	leaq		0(%r10, %r11, 4), %r13
	leaq		0(%r10, %r11, 8), %r14
#if 0
	prefetcht1	0(%r10)				// 0
	prefetcht1	0(%r10, %r11)	// 1
	prefetcht1	0(%r10, %r11, 2)	// 2
	prefetcht1	0(%r10, %r12)	// 3
	prefetcht1	0(%r13)				// 4
	prefetcht1	0(%r13, %r11)	// 5
	prefetcht1	0(%r13, %r11, 2)	// 6
	prefetcht1	0(%r13, %r12)	// 7
	prefetcht1	0(%r14)				// 8
	prefetcht1	0(%r14, %r11)	// 9
	prefetcht1	0(%r14, %r11, 2)	// 10
	prefetcht1	0(%r14, %r12)	// 11
#else
	prefetcht1	31(%r10)				// 0
	prefetcht1	31(%r10, %r11)	// 1
	prefetcht1	31(%r10, %r11, 2)	// 2
	prefetcht1	31(%r10, %r12)	// 3
	prefetcht1	31(%r13)				// 4
	prefetcht1	31(%r13, %r11)	// 5
	prefetcht1	31(%r13, %r11, 2)	// 6
	prefetcht1	31(%r13, %r12)	// 7
	prefetcht1	31(%r14)				// 8
	prefetcht1	31(%r14, %r11)	// 9
	prefetcht1	31(%r14, %r11, 2)	// 10
	prefetcht1	31(%r14, %r12)	// 11
#endif

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_prefetch1_4x12_lib)
#endif





//                                  1      2              3          4        5          6        7             8          9        10         11
// void kernel_dgemm_nt_12x4_libcccc(int k, double *alpha, double *A, int lda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_12x4_libcccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // lda
	sall	$ 3, %r12d
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIBCC
#else
	CALL(inner_kernel_dgemm_nt_12x4_libcc)
#endif


	// prefetch D

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_12X4_LIB
#else
	CALL(inner_prefetch0_12x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_12x4_libcccc)





//                                     1      2              3          4        5          6        7             8          9        10         11       12      13
// void kernel_dgemm_nt_12x4_vs_libcccc(int k, double *alpha, double *A, int lda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_12x4_vs_libcccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // lda
	sall	$ 3, %r12d // lda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d
	movq	ARG12, %r15  // m1

	movq	ARG13, %rax  // n1
	cmpl	$ 1, %eax
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X1_VS_LIBCC
#else
	CALL(inner_kernel_dgemm_nt_12x1_vs_libcc)
#endif
	
	jmp		103f

100:

	movq	ARG13, %rax  // n1
	cmpl	$ 2, %eax
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X2_VS_LIBCC
#else
	CALL(inner_kernel_dgemm_nt_12x2_vs_libcc)
#endif

	jmp		103f

101:

	movq	ARG13, %rax  // n1
	cmpl	$ 3, %eax
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X3_VS_LIBCC
#else
	CALL(inner_kernel_dgemm_nt_12x3_vs_libcc)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_VS_LIBCC
#else
	CALL(inner_kernel_dgemm_nt_12x4_vs_libcc)
#endif

103:


	// prefetch D
	// TODO prefetch vs !!!

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_12X4_LIB
#else
	CALL(inner_prefetch0_12x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG12, %r14 // m1
	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_12x4_vs_lib4ccc)





//                                  1      2              3          4        5          6        7             8          9        10         11
// void kernel_dgemm_nt_12x4_lib4ccc(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_12x4_lib4ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// prefetch C

	movq		ARG7, %r10 // beta
	vmovsd		0(%r10), %xmm14
	vxorpd		%xmm15, %xmm15, %xmm15 // 0.0
	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			100f // end

	movq	ARG8, %r10 // C
	movq	ARG9, %r11 // ldc
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_12X4_LIB
#else
	CALL(inner_prefetch0_12x4_lib)
#endif

100:


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif


	// prefetch D

//	movq	ARG10, %r10 // D
//	movq	ARG11, %r11 // ldd
//	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
//	INNER_PREFETCH0_12X4_LIB
#else
//	CALL(inner_prefetch0_12x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_12x4_lib4ccc)





//                                     1      2              3          4        5          6        7             8          9        10         11       12      13
// void kernel_dgemm_nt_12x4_vs_lib4ccc(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_12x4_vs_lib4ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

	movq	ARG13, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG13, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG13, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif

103:


	// prefetch D
	// TODO prefetch vs !!!

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_12X4_LIB
#else
	CALL(inner_prefetch0_12x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG12, %r14 // m1
	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_12x4_vs_lib4ccc)





//                                  1      2              3          4        5          6             7          8        9          10
// void kernel_dgemm_nt_12x4_lib44cc(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_12x4_lib44cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// prefetch C

	prefetcht1	0(ARG2) // alpha
	movq		ARG6, %r10 // beta
	vmovsd		0(%r10), %xmm14
	vxorpd		%xmm15, %xmm15, %xmm15 // 0.0
	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			100f // end

	movq	ARG7, %r10 // C
	movq	ARG8, %r11 // ldc
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH1_12X4_LIB
#else
	CALL(inner_prefetch1_12x4_lib)
#endif

100:


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_12x4_lib44cc)





//                                      1      2              3          4        5          6             7          8        9          10       11            12
// void kernel_dgemm_nt_12x4_p0_lib44cc(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *pA_p, double *pB_p);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_12x4_p0_lib44cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// prefetch C

	prefetcht1	0(ARG2) // alpha
	movq		ARG6, %r10 // beta
	vmovsd		0(%r10), %xmm14
	vxorpd		%xmm15, %xmm15, %xmm15 // 0.0
	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			100f // end

	movq	ARG7, %r10 // C
	movq	ARG8, %r11 // ldc
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH1_12X4_LIB
#else
	CALL(inner_prefetch1_12x4_lib)
#endif

100:


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B

	movq	ARG11, %r14 // A_p
	addq	ARG12, %r15 // B_p

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_P0_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_p0_lib4)
#endif



	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_12x4_p0_lib44cc)





//                                     1      2              3          4        5          6             7          8        9          10       11      12
// void kernel_dgemm_nt_12x4_vs_lib44cc(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_12x4_vs_lib44cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// prefetch C

	prefetcht1	0(ARG2) // alpha
	movq		ARG6, %r10 // beta
	vmovsd		0(%r10), %xmm14
	vxorpd		%xmm15, %xmm15, %xmm15 // 0.0
	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			100f // end

	movq	ARG7, %r10 // C
	movq	ARG8, %r11 // ldc
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH1_12X4_LIB
#else
	CALL(inner_prefetch1_12x4_lib)
#endif

100:


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG11, %r14 // ldd
	movq	ARG12, %r15 // ldd

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG11, %r12 // ldd
	movq	ARG12, %r13 // ldd

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_12x4_vs_lib44cc)





//                                  1      2              3          4        5          6             7          8        9          10
// void kernel_dgemm_nt_12x4_libc4cc(int k, double *alpha, double *A, int lda, double *B, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_12x4_libc4cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


#if 1

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // lda
	sall	$ 3, %r12d
	movq	ARG5, %r13  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIBC4
#else
	CALL(inner_kernel_dgemm_nt_12x4_libc4)
#endif

#else

	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG5, %r11  // B
	movq	ARG3, %r12  // A
	movq	ARG4, %r13  // lda
	sall	$ 3, %r13d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X12_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_4x12_lib4c)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif

#endif

	// prefetch D

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_12X4_LIB
#else
	CALL(inner_prefetch0_12x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_12x4_libc4cc)





//                                    1      2              3          4        5          6             7          8        9          10       11      12
// void kernel_dgemm_nt_12x4_vs_libc4cc(int k, double *alpha, double *A, int lda, double *B, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_12x4_vs_libc4cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

#if 1

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // lda
	sall	$ 3, %r12d
	movq	ARG5, %r13  // B

	movq	ARG12, %r14  // n1
	cmpl	$ 1, %r14d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X1_LIBC4
#else
	CALL(inner_kernel_dgemm_nt_12x1_libc4)
#endif
	
	jmp		103f

100:

	movq	ARG12, %r14  // n1
	cmpl	$ 2, %r14d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X2_LIBC4
#else
	CALL(inner_kernel_dgemm_nt_12x2_libc4)
#endif

	jmp		103f

101:

	movq	ARG12, %r14  // n1
	cmpl	$ 3, %r14d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X3_LIBC4
#else
	CALL(inner_kernel_dgemm_nt_12x3_libc4)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIBC4
#else
	CALL(inner_kernel_dgemm_nt_12x4_libc4)
#endif

103:

#else

	movq	ARG1, %r10 // k
	movq	ARG5, %r11  // B
	movq	ARG3, %r12  // A
	movq	ARG4, %r13  // lda
	sall	$ 3, %r13d

	movq	ARG11, %r14  // m1
	cmpl	$ 9, %r14d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X9_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_4x9_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG11, %r14  // m1
	cmpl	$ 10, %r14d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X10_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_4x10_lib4c)
#endif

	jmp		103f

101:

	movq	ARG11, %r14  // m1
	cmpl	$ 11, %r14d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X11_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_4x11_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X12_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_4x12_lib4c)
#endif

103:

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif

#endif


	// prefetch D
	// TODO prefetch vs !!!

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_12X4_LIB
#else
	CALL(inner_prefetch0_12x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG11, %r14 // m1
	movq	ARG12, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG11, %r12 // m1
	movq	ARG12, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_12x4_vs_libc4cc)





//                                      1      2              3          4        5          6             7          8        9          10       11            12
// void kernel_dgemm_nt_4x12_p0_lib44cc(int k, double *alpha, double *A, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd, double *pA_p, double *pB_p);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_4x12_p0_lib44cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// prefetch C

	prefetcht1	0(ARG2) // alpha
	movq		ARG6, %r10 // beta
	vmovsd		0(%r10), %xmm14
	vxorpd		%xmm15, %xmm15, %xmm15 // 0.0
	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			100f // end

	movq	ARG7, %r10 // C
	movq	ARG8, %r11 // ldc
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH1_4X12_LIB
#else
	CALL(inner_prefetch1_4x12_lib)
#endif

100:


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG4, %r11  // B
	movq	ARG5, %r12 // sdb
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG3, %r13  // A

	movq	ARG12, %r14 // B_p
	addq	ARG11, %r15 // A_p

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_P0_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_p0_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X12_LIB
#else
	CALL(inner_scale_ab_4x12_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_LIB
#else
	CALL(inner_store_4x12_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_4x12_p0_lib44cc)





//                                  1      2              3          4        5          6             7          8        9          10
// void kernel_dgemm_nt_4x12_lib44cc(int k, double *alpha, double *A, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_4x12_lib44cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// prefetch C

	prefetcht1	0(ARG2) // alpha
	movq		ARG6, %r10 // beta
	vmovsd		0(%r10), %xmm14
	vxorpd		%xmm15, %xmm15, %xmm15 // 0.0
	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			100f // end

	movq	ARG7, %r10 // C
	movq	ARG8, %r11 // ldc
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH1_4X12_LIB
#else
	CALL(inner_prefetch1_4x12_lib)
#endif

100:


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG4, %r11  // B
	movq	ARG5, %r12 // sdb
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG3, %r13  // A

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X12_LIB
#else
	CALL(inner_scale_ab_4x12_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_LIB
#else
	CALL(inner_store_4x12_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_4x12_lib44cc)





//                                     1      2              3          4        5          6             7          8        9          10       11      12
// void kernel_dgemm_nt_4x12_vs_lib44cc(int k, double *alpha, double *A, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_4x12_vs_lib44cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG4, %r11  // B
	movq	ARG5, %r12 // sdb
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG3, %r13  // A

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG11, %r14 // ldd
	movq	ARG12, %r15 // ldd

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X12_VS_LIB
#else
	CALL(inner_scale_ab_4x12_vs_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG11, %r12 // ldd
	movq	ARG12, %r13 // ldd

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_VS_LIB
#else
	CALL(inner_store_4x12_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_4x12_vs_lib44cc)





//                                  1      2              3          4        5          6        7             8          9        10         11
// void kernel_dgemm_nn_12x4_libcccc(int k, double *alpha, double *A, int lda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nn_12x4_libcccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // lda
	sall	$ 3, %r12d
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIBCC
#else
	CALL(inner_kernel_dgemm_nn_12x4_libcc)
#endif


	// prefetch D

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_12X4_LIB
#else
	CALL(inner_prefetch0_12x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nn_12x4_libcccc)





//                                    1      2              3          4        5          6        7             8          9        10         11        12      13
// void kernel_dgemm_nn_12x4_vs_libcccc(int k, double *alpha, double *A, int lda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nn_12x4_vs_libcccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 3, %r12d // sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb*sizeof(double)
	sall	$ 3, %r14d
	movq	ARG12, %r15  // m1

	movq	ARG13, %rax  // n1
	cmpl	$ 1, %eax
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X1_VS_LIBCC
#else
	CALL(inner_kernel_dgemm_nn_12x1_vs_libcc)
#endif
	
	jmp		103f

100:

	movq	ARG13, %rax  // n1
	cmpl	$ 2, %eax
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X2_VS_LIBCC
#else
	CALL(inner_kernel_dgemm_nn_12x2_vs_libcc)
#endif

	jmp		103f

101:

	movq	ARG13, %rax  // n1
	cmpl	$ 3, %eax
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X3_VS_LIBCC
#else
	CALL(inner_kernel_dgemm_nn_12x3_vs_libcc)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_VS_LIBCC
#else
	CALL(inner_kernel_dgemm_nn_12x4_vs_libcc)
#endif

103:


	// prefetch D
	// TODO prefetch vs !!!

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_12X4_LIB
#else
	CALL(inner_prefetch0_12x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG12, %r14 // m1
	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nn_12x4_vs_libcccc)





//                                 1      2              3          4        5          6        7             8          9        10         11
// void kernel_dgemm_nn_12x4_lib4ccc(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nn_12x4_lib4ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// prefetch C

	movq		ARG7, %r10 // beta
	vmovsd		0(%r10), %xmm14
	vxorpd		%xmm15, %xmm15, %xmm15 // 0.0
	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			100f // end

	movq	ARG8, %r10 // C
	movq	ARG9, %r11 // ldc
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_12X4_LIB
#else
	CALL(inner_prefetch0_12x4_lib)
#endif

100:


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif


	// prefetch D

//	movq	ARG10, %r10 // D
//	movq	ARG11, %r11 // ldd
//	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
//	INNER_PREFETCH0_12X4_LIB
#else
//	CALL(inner_prefetch0_12x4_lib)
#endif


	movq	ARG10, %r10 // D
	cmpq	ARG8, %r10
	jne			100f // end

	movq	ARG11, %r10 // ldd
	cmpq	ARG9, %r10
	jne			100f // end

	movq	ARG7, %r10 // beta
	vbroadcastsd	0(%r10), %ymm14
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC05(%rip), %ymm15 // 1.0
#else
	vmovapd		LC05(%rip), %ymm15 // 1.0
#endif
	vucomisd	%xmm15, %xmm14 // beta==1.0 ?
	jne			100f // end



	movq	ARG2, %r10 // alpha
	movq	ARG10, %r11   // D
	movq	ARG11, %r12   // ldd
	sall	$ 3, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_A1_STORE_12X4_LIB
#else
	CALL(inner_scale_a1_store_12x4_lib)
#endif


	jmp		101f


100:


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


101:


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nn_12x4_lib4ccc)





//                                    1      2              3          4        5          6        7             8          9        10         11        12      13
// void kernel_dgemm_nn_12x4_vs_lib4ccc(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nn_12x4_vs_lib4ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

	movq	ARG13, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG13, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG13, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif

103:


	// prefetch D
	// TODO prefetch vs !!!

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_12X4_LIB
#else
	CALL(inner_prefetch0_12x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG12, %r14 // m1
	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nn_12x4_vs_lib4ccc)





//                                  1      2              3          4        5          6        7             8          9        10         11
// void kernel_dgemm_tt_4x12_libcccc(int k, double *alpha, double *A, int lda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_tt_4x12_libcccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG5, %r11  // B
	movq	ARG6, %r12  // ldb
	sall	$ 3, %r12d
	movq	ARG3, %r13  // A
	movq	ARG4, %r14 // lda
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIBCC
#else
	CALL(inner_kernel_dgemm_nn_12x4_libcc)
#endif


	// prefetch D

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_4X12_LIB
#else
	CALL(inner_prefetch0_4x12_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif


#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X12_LIB
#else
	CALL(inner_scale_ab_4x12_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_LIB
#else
	CALL(inner_store_4x12_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_tt_4x12_libcccc)





//                                    1      2              3          4        5          6        7             8          9        10         11        12      13
// void kernel_dgemm_tt_4x12_vs_libcccc(int k, double *alpha, double *A, int lda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_tt_4x12_vs_libcccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG5, %r11  // B
	movq	ARG6, %r12  // ldb*sizeof(double)
	sall	$ 3, %r12d
	movq	ARG3, %r13  // A
	movq	ARG4, %r14 // sda
	sall	$ 3, %r14d // sda*sizeof(double)
	movq	ARG13, %r15  // n1

	movq	ARG12, %rax  // m1
	cmpl	$ 1, %eax
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X1_VS_LIBCC
#else
	CALL(inner_kernel_dgemm_nn_12x1_vs_libcc)
#endif
	
	jmp		103f

100:

	movq	ARG12, %rax  // m1
	cmpl	$ 2, %eax
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X2_VS_LIBCC
#else
	CALL(inner_kernel_dgemm_nn_12x2_vs_libcc)
#endif

	jmp		103f

101:

	movq	ARG12, %rax  // m1
	cmpl	$ 3, %eax
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X3_VS_LIBCC
#else
	CALL(inner_kernel_dgemm_nn_12x3_vs_libcc)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_VS_LIBCC
#else
	CALL(inner_kernel_dgemm_nn_12x4_vs_libcc)
#endif

103:


	// prefetch D
	// TODO prefetch vs !!!

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_4X12_LIB
#else
	CALL(inner_prefetch0_4x12_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG12, %r14 // m1
	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif


#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X12_VS_LIB
#else
	CALL(inner_scale_ab_4x12_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_VS_LIB
#else
	CALL(inner_store_4x12_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_tt_4x12_vs_libcccc)





//                                  1      2              3          4        5          6             7          8        9          10
// void kernel_dgemm_tt_12x4_libc4cc(int k, double *alpha, double *A, int lda, double *B, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_tt_12x4_libc4cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// prefetch C

	movq		ARG6, %r10 // beta
	vmovsd		0(%r10), %xmm14
	vxorpd		%xmm15, %xmm15, %xmm15 // 0.0
	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			100f // end

	movq	ARG7, %r10 // C
	movq	ARG8, %r11 // ldc
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_12X4_LIB
#else
	CALL(inner_prefetch0_12x4_lib)
#endif

100:

	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG5, %r11  // B
	movq	ARG3, %r12  // A
	movq	ARG4, %r13  // lda
	sall	$ 3, %r13d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X12_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_4x12_lib4c)
#endif


	// prefetch D

//	movq	ARG9, %r10 // D
//	movq	ARG10, %r11 // ldd
//	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
//	INNER_PREFETCH0_12X4_LIB
#else
//	CALL(inner_prefetch0_12x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif


#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_tt_12x4_libc4cc)





//                                    1      2              3          4        5          6             7          8        9          10       11      12
// void kernel_dgemm_tt_12x4_vs_libc4cc(int k, double *alpha, double *A, int lda, double *B, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_tt_12x4_vs_libc4cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG5, %r11  // B
	movq	ARG3, %r12  // A
	movq	ARG4, %r13  // lda
	sall	$ 3, %r13d

	movq	ARG11, %r14  // m1
	cmpl	$ 9, %r14d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X9_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_4x9_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG11, %r14  // m1
	cmpl	$ 10, %r14d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X10_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_4x10_lib4c)
#endif

	jmp		103f

101:

	movq	ARG11, %r14  // m1
	cmpl	$ 11, %r14d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X11_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_4x11_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X12_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_4x12_lib4c)
#endif

103:


	// prefetch D
	// TODO prefetch vs !!!

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_12X4_LIB
#else
	CALL(inner_prefetch0_12x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG11, %r14 // m1
	movq	ARG12, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif


#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG11, %r12 // m1
	movq	ARG12, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_tt_12x4_vs_libc4cc)





//                                  1      2              3          4        5          6        7             8          9        10         11
// void kernel_dgemm_tt_12x4_libcccc(int k, double *alpha, double *A, int lda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_tt_12x4_libcccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG5, %r11  // B
	movq	ARG6, %r12  // ldb
	sall	$ 3, %r12d
	movq	ARG3, %r13  // A
	movq	ARG4, %r14 // lda
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X12_LIBCC
#else
	CALL(inner_kernel_dgemm_nn_4x12_libcc)
#endif


	// prefetch D

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_12X4_LIB
#else
	CALL(inner_prefetch0_12x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif


#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_tt_12x4_libcccc)





//                                    1      2              3          4        5          6        7             8          9        10         11        12      13
// void kernel_dgemm_tt_12x4_vs_libcccc(int k, double *alpha, double *A, int lda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_tt_12x4_vs_libcccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG5, %r11  // B
	movq	ARG6, %r12  // ldb*sizeof(double)
	sall	$ 3, %r12d
	movq	ARG3, %r13  // A
	movq	ARG4, %r14 // sda
	sall	$ 3, %r14d // sda*sizeof(double)
	movq	ARG13, %r15  // n1

	movq	ARG12, %rax  // m1
	cmpl	$ 9, %eax
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X9_VS_LIBCC
#else
	CALL(inner_kernel_dgemm_nn_4x9_vs_libcc)
#endif
	
	jmp		103f

100:

	movq	ARG12, %rax  // m1
	cmpl	$ 10, %eax
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X10_VS_LIBCC
#else
	CALL(inner_kernel_dgemm_nn_4x10_vs_libcc)
#endif

	jmp		103f

101:

	movq	ARG12, %rax  // m1
	cmpl	$ 11, %eax
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X11_VS_LIBCC
#else
	CALL(inner_kernel_dgemm_nn_4x11_vs_libcc)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X12_VS_LIBCC
#else
	CALL(inner_kernel_dgemm_nn_4x12_vs_libcc)
#endif

103:


	// prefetch D
	// TODO prefetch vs !!!

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_12X4_LIB
#else
	CALL(inner_prefetch0_12x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG12, %r14 // m1
	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif


#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_tt_12x4_vs_libcccc)





//                                 1      2              3          4        5          6        7             8          9        10         11
// void kernel_dsyrk_nt_l_12x4_lib4ccc(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nt_l_12x4_lib4ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif


	// prefetch D

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_12X4_LIB
#else
	CALL(inner_prefetch0_12x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_L_12X4_LIB
#else
	CALL(inner_store_l_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_nt_l_12x4_lib4ccc)





//                                    1      2              3          4        5          6        7             8          9        10         11       12      13
// void kernel_dsyrk_nt_l_12x4_vs_lib4ccc(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nt_l_12x4_vs_lib4ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

	movq	ARG13, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG13, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG13, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif

103:


	// prefetch D
	// TODO prefetch vs !!!

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_12X4_LIB
#else
	CALL(inner_prefetch0_12x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG12, %r14 // m1
	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_12X4_VS_LIB
#else
	CALL(inner_store_l_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_nt_l_12x4_vs_lib4ccc)





//                                    1      2              3          4        5          6             7          8        9          10
// void kernel_dsyrk_nt_l_12x4_lib44cc(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nt_l_12x4_lib44cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_L_12X4_LIB
#else
	CALL(inner_store_l_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_nt_l_12x4_lib44cc)





//                                        1      2              3          4        5          6             7          8        9          10       11      12
// void kernel_dsyrk_nt_l_12x4_vs_lib44cc(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nt_l_12x4_vs_lib44cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG11, %r14 // ldd
	movq	ARG12, %r15 // ldd

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG11, %r12 // ldd
	movq	ARG12, %r13 // ldd

#if MACRO_LEVEL>=1
	INNER_STORE_L_12X4_VS_LIB
#else
	CALL(inner_store_l_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_nt_l_12x4_vs_lib44cc)





//                                    1      2              3          4        5          6             7          8        9          10
// void kernel_dsyrk_nt_u_12x4_lib44cc(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nt_u_12x4_lib44cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_U_12X4_LIB
#else
	CALL(inner_store_u_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_nt_u_12x4_lib44cc)





//                                        1      2              3          4        5          6             7          8        9          10       11      12
// void kernel_dsyrk_nt_u_12x4_vs_lib44cc(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nt_u_12x4_vs_lib44cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG11, %r14 // ldd
	movq	ARG12, %r15 // ldd

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG11, %r12 // ldd
	movq	ARG12, %r13 // ldd

#if MACRO_LEVEL>=1
	INNER_STORE_U_12X4_VS_LIB
#else
	CALL(inner_store_u_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_nt_u_12x4_vs_lib44cc)





//                                    1      2              3           4         5           6         7           8         9           10        11            12         13       14         15
// void kernel_dger2k_nt_12x4_lib4ccc(int k, double *alpha, double *A0, int sda0, double *B0, int ldb0, double *A1, int sda1, double *B1, int ldb1, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dger2k_nt_12x4_lib4ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A0
	movq	ARG4, %r12 // sda0
	sall	$ 5, %r12d // 4*sda0*sizeof(double)
	movq	ARG5, %r13  // B0
	movq	ARG6, %r14  // ldb0
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif

	movq	ARG1, %r10 // k
	movq	ARG7, %r11  // A1
	movq	ARG8, %r12 // sda1
	sall	$ 5, %r12d // 4*sda1*sizeof(double)
	movq	ARG9, %r13  // B1
	movq	ARG10, %r14  // ldb1
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif


	// prefetch D

	movq	ARG14, %r10 // D
	movq	ARG15, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_12X4_LIB
#else
	CALL(inner_prefetch0_12x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG11, %r11 // beta
	movq	ARG12, %r12   // C
	movq	ARG13, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG14, %r10 // D
	movq	ARG15, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dger2k_nt_12x4_lib4ccc)





//                                      1      2              3           4         5           6         7           8         9           10        11            12         13       14         15       16      17
// void kernel_dger2k_nt_12x4_vs_lib4ccc(int k, double *alpha, double *A0, int sda0, double *B0, int ldb0, double *A1, int sda1, double *B1, int ldb1, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dger2k_nt_12x4_vs_lib4ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG17, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A0
	movq	ARG4, %r12 // sda0
	sall	$ 5, %r12d // 4*sda0*sizeof(double)
	movq	ARG5, %r13  // B0
	movq	ARG6, %r14  // ldb0
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x1_lib4c)
#endif

	movq	ARG1, %r10 // k
	movq	ARG7, %r11  // A1
	movq	ARG8, %r12 // sda1
	sall	$ 5, %r12d // 4*sda1*sizeof(double)
	movq	ARG9, %r13  // B1
	movq	ARG10, %r14  // ldb1
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x1_lib4c)
#endif

	jmp		103f

100:

	movq	ARG17, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A0
	movq	ARG4, %r12 // sda0
	sall	$ 5, %r12d // 4*sda0*sizeof(double)
	movq	ARG5, %r13  // B0
	movq	ARG6, %r14  // ldb0
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x2_lib4c)
#endif

	movq	ARG1, %r10 // k
	movq	ARG7, %r11  // A1
	movq	ARG8, %r12 // sda1
	sall	$ 5, %r12d // 4*sda1*sizeof(double)
	movq	ARG9, %r13  // B1
	movq	ARG10, %r14  // ldb1
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG17, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A0
	movq	ARG4, %r12 // sda0
	sall	$ 5, %r12d // 4*sda0*sizeof(double)
	movq	ARG5, %r13  // B0
	movq	ARG6, %r14  // ldb0
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x3_lib4c)
#endif

	movq	ARG1, %r10 // k
	movq	ARG7, %r11  // A1
	movq	ARG8, %r12 // sda1
	sall	$ 5, %r12d // 4*sda1*sizeof(double)
	movq	ARG9, %r13  // B1
	movq	ARG10, %r14  // ldb1
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x3_lib4c)
#endif

	jmp		103f

102:

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A0
	movq	ARG4, %r12 // sda0
	sall	$ 5, %r12d // 4*sda0*sizeof(double)
	movq	ARG5, %r13  // B0
	movq	ARG6, %r14  // ldb0
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif

	movq	ARG1, %r10 // k
	movq	ARG7, %r11  // A1
	movq	ARG8, %r12 // sda1
	sall	$ 5, %r12d // 4*sda1*sizeof(double)
	movq	ARG9, %r13  // B1
	movq	ARG10, %r14  // ldb1
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif

103:


	// prefetch D
	// TODO prefetch vs !!!

	movq	ARG14, %r10 // D
	movq	ARG15, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_12X4_LIB
#else
	CALL(inner_prefetch0_12x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG11, %r11 // beta
	movq	ARG12, %r12   // C
	movq	ARG13, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG16, %r14 // m1
	movq	ARG17, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif


	// store n

	movq	ARG14, %r10 // D
	movq	ARG15, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG16, %r12 // m1
	movq	ARG17, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dger2k_nt_12x4_vs_lib4ccc)





//                                   1      2               3          4         5           6           7         8           9             10         11       12         13
// void kernel_dger2k_nt_12x4_lib44cc(int k, double *alpha, double *A0, int sda0, double *B0, double *A1, int sda1, double *B1, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dger2k_nt_12x4_lib44cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// prefetch C

	movq		ARG9, %r10 // beta
	vmovsd		0(%r10), %xmm14
	vxorpd		%xmm15, %xmm15, %xmm15 // 0.0
	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			100f // end

	movq	ARG10, %r10 // C
	movq	ARG11, %r11 // ldc
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_12X4_LIB
#else
	CALL(inner_prefetch0_12x4_lib)
#endif

100:


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A0
	movq	ARG4, %r12 // sda0
	sall	$ 5, %r12d // 4*sda0*sizeof(double)
	movq	ARG5, %r13  // B0

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif

	movq	ARG1, %r10 // k
	movq	ARG6, %r11  // A1
	movq	ARG7, %r12 // sda1
	sall	$ 5, %r12d // 4*sda1*sizeof(double)
	movq	ARG8, %r13  // B1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG9, %r11 // beta
	movq	ARG10, %r12   // C
	movq	ARG11, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG12, %r10 // D
	movq	ARG13, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dger2k_nt_12x4_lib44cc)





//                                      1      2               3          4         5           6           7         8           9             10         11       12         13       14      15
// void kernel_dger2k_nt_12x4_vs_lib44cc(int k, double *alpha, double *A0, int sda0, double *B0, double *A1, int sda1, double *B1, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dger2k_nt_12x4_vs_lib44cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A0
	movq	ARG4, %r12 // sda0
	sall	$ 5, %r12d // 4*sda0*sizeof(double)
	movq	ARG5, %r13  // B0

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif

	movq	ARG1, %r10 // k
	movq	ARG6, %r11  // A1
	movq	ARG7, %r12 // sda1
	sall	$ 5, %r12d // 4*sda1*sizeof(double)
	movq	ARG8, %r13  // B1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG9, %r11 // beta
	movq	ARG10, %r12   // C
	movq	ARG11, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG14, %r14 // m1
	movq	ARG15, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif


	// store n

	movq	ARG12, %r10 // D
	movq	ARG13, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG14, %r12 // m1
	movq	ARG15, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dger2k_nt_12x4_vs_lib44cc)





//                                     1      2              3           4         5           6           7         8           9             10         11       12         13
// void kernel_dsyr2k_nt_l_12x4_lib44cc(int k, double *alpha, double *A0, int sda0, double *B0, double *A1, int sda1, double *B1, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyr2k_nt_l_12x4_lib44cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A0
	movq	ARG4, %r12 // sda0
	sall	$ 5, %r12d // 4*sda0*sizeof(double)
	movq	ARG5, %r13  // B0

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif

	movq	ARG1, %r10 // k
	movq	ARG6, %r11  // A1
	movq	ARG7, %r12 // sda1
	sall	$ 5, %r12d // 4*sda1*sizeof(double)
	movq	ARG8, %r13  // B1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG9, %r11 // beta
	movq	ARG10, %r12   // C
	movq	ARG11, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG12, %r10 // D
	movq	ARG13, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_L_12X4_LIB
#else
	CALL(inner_store_l_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dsyr2k_nt_l_12x4_lib44cc)





//                                        1      2              3           4         5           6           7         8           9             10         11       12         13       14      15
// void kernel_dsyr2k_nt_l_12x4_vs_lib44cc(int k, double *alpha, double *A0, int sda0, double *B0, double *A1, int sda1, double *B1, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyr2k_nt_l_12x4_vs_lib44cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A0
	movq	ARG4, %r12 // sda0
	sall	$ 5, %r12d // 4*sda0*sizeof(double)
	movq	ARG5, %r13  // B0

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif

	movq	ARG1, %r10 // k
	movq	ARG6, %r11  // A1
	movq	ARG7, %r12 // sda1
	sall	$ 5, %r12d // 4*sda1*sizeof(double)
	movq	ARG8, %r13  // B1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG9, %r11 // beta
	movq	ARG10, %r12   // C
	movq	ARG11, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG14, %r14 // m1
	movq	ARG15, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif


	// store n

	movq	ARG12, %r10 // D
	movq	ARG13, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG14, %r12 // m1
	movq	ARG15, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_12X4_VS_LIB
#else
	CALL(inner_store_l_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dsyr2k_nt_l_12x4_vs_lib44cc)





//                                 1      2              3          4        5          6        7             8          9        10         11
// void kernel_dtrmm_nn_rl_12x4_lib4ccc(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd,);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_rl_12x4_lib4ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NN_RL_12X4_LIB4C
#else
	CALL(inner_edge_dtrmm_nn_rl_12x4_lib4c)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nn_rl_12x4_lib4ccc)





//                                    1      2              3          4        5          6        7             8          9        10         11       12      13
// void kernel_dtrmm_nn_rl_12x4_vs_lib4ccc(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_rl_12x4_vs_lib4ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NN_RL_12X4_VS_LIB4C
#else
	CALL(inner_edge_dtrmm_nn_rl_12x4_vs_lib4c)
#endif

	movq	ARG13, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG13, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG13, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif

103:


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG12, %r14 // m1
	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nn_rl_12x4_vs_lib4ccc)





//                                         1      2              3          4        5          6        7             8          9        10         11
// void kernel_dtrmm_nn_rl_one_12x4_lib4ccc(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd,);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_rl_one_12x4_lib4ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NN_RL_ONE_12X4_LIB4C
#else
	CALL(inner_edge_dtrmm_nn_rl_one_12x4_lib4c)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nn_rl_one_12x4_lib4ccc)





//                                            1      2              3          4        5          6        7             8          9        10         11       12      13
// void kernel_dtrmm_nn_rl_one_12x4_vs_lib4ccc(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_rl_one_12x4_vs_lib4ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NN_RL_ONE_12X4_VS_LIB4C
#else
	CALL(inner_edge_dtrmm_nn_rl_one_12x4_vs_lib4c)
#endif

	movq	ARG13, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG13, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG13, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif

103:


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG12, %r14 // m1
	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nn_rl_one_12x4_vs_lib4ccc)





//                                          1      2              3          4          5        6             7          8        9
// void kernel_dtrmm_nn_rl_4x12_tran_lib4c4c(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_rl_4x12_tran_lib4c4c)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NN_RL_12X4_LIB4C
#else
	CALL(inner_edge_dtrmm_nn_rl_12x4_lib4c)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_LIB
#else
	CALL(inner_store_4x12_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nn_rl_4x12_tran_lib4c4c)





//                                             1      2              3          4          5        6             7          8        9          10       11      12
// void kernel_dtrmm_nn_rl_4x8_tran_vs_lib4c4c(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_rl_4x12_tran_vs_lib4c4c)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NN_RL_12X4_VS_LIB4C
#else
	CALL(inner_edge_dtrmm_nn_rl_12x4_vs_lib4c)
#endif

	movq	ARG12, %r15  // m1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG12, %r15  // m1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG12, %r15  // m1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif

103:


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_VS_LIB
#else
	CALL(inner_store_4x12_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nn_rl_4x12_tran_vs_lib4c4c)





//                                          1      2              3          4          5        6             7          8        9
// void kernel_dtrmm_nn_rl_one_4x12_tran_lib4c4c(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_rl_one_4x12_tran_lib4c4c)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NN_RL_ONE_12X4_LIB4C
#else
	CALL(inner_edge_dtrmm_nn_rl_one_12x4_lib4c)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_LIB
#else
	CALL(inner_store_4x12_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nn_rl_one_4x12_tran_lib4c4c)





//                                             1      2              3          4          5        6             7          8        9          10       11      12
// void kernel_dtrmm_nn_rl_one_4x12_tran_vs_lib4c4c(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_rl_one_4x12_tran_vs_lib4c4c)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NN_RL_ONE_12X4_VS_LIB4C
#else
	CALL(inner_edge_dtrmm_nn_rl_one_12x4_vs_lib4c)
#endif

	movq	ARG12, %r15  // m1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG12, %r15  // m1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG12, %r15  // m1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif

103:


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_VS_LIB
#else
	CALL(inner_store_4x12_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nn_rl_one_4x12_tran_vs_lib4c4c)





//                                    1      2              3          4          5        6             7          8        9          10        11
// void kernel_dtrmm_nn_ru_12x4_lib4ccc(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_ru_12x4_lib4ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NN_RU_12X4_LIB4C
#else
	CALL(inner_edge_dtrmm_nn_ru_12x4_lib4c)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nn_ru_12x4_lib4ccc)





//                                       1      2              3          4          5        6             7          8        9          10       11      12       13
// void kernel_dtrmm_nn_ru_12x4_vs_lib4ccc(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_ru_12x4_vs_lib4ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

	movq	ARG13, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG13, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG13, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif

103:

	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NN_RU_12X4_VS_LIB4C
#else
	CALL(inner_edge_dtrmm_nn_ru_12x4_vs_lib4c)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG12, %r14 // m1
	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nn_ru_12x4_vs_lib4ccc)





//                                          1      2              3          4         5         6        7             8          9        10         11
// void kernel_dtrmm_nn_ru_one_12x4_lib4ccc(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_ru_one_12x4_lib4ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NN_RU_ONE_12X4_LIB4C
#else
	CALL(inner_edge_dtrmm_nn_ru_one_12x4_lib4c)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nn_ru_one_12x4_lib4ccc)





//                                            1      2              3          4          5        6             7          8        9          10       11      12       13
// void kernel_dtrmm_nn_ru_one_12x4_vs_lib4ccc(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_ru_one_12x4_vs_lib4ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

	movq	ARG13, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG13, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG13, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif

103:

	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NN_RU_ONE_12X4_VS_LIB4C
#else
	CALL(inner_edge_dtrmm_nn_ru_one_12x4_vs_lib4c)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG12, %r14 // m1
	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nn_ru_one_12x4_vs_lib4ccc)





//                                    1      2              3          4          5        6             7          8        9          10  11         12
// void kernel_dtrmm_nn_ru_4x12_tran_lib4c4c(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_ru_4x12_tran_lib4c4c)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NN_RU_12X4_LIB4C
#else
	CALL(inner_edge_dtrmm_nn_ru_12x4_lib4c)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_LIB
#else
	CALL(inner_store_4x12_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nn_ru_4x12_tran_lib4c4c)





//                                             1      2              3          4          5        6             7          8        9          10       11      12     13
// void kernel_dtrmm_nn_ru_4x12_tran_vs_lib4c4c(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_ru_4x12_tran_vs_lib4c4c)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

	movq	ARG12, %r15  // m1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG12, %r15  // m1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG12, %r15  // m1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif

103:

	movq	ARG12, %r15 // m1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NN_RU_12X4_VS_LIB4C
#else
	CALL(inner_edge_dtrmm_nn_ru_12x4_vs_lib4c)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_VS_LIB
#else
	CALL(inner_store_4x12_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nn_ru_4x12_tran_vs_lib4c4c)





//                                    1      2              3          4          5        6             7          8        9          10  11         12
// void kernel_dtrmm_nn_ru_one_4x8_tran_lib4c4c(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_ru_one_4x12_tran_lib4c4c)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NN_RU_ONE_12X4_LIB4C
#else
	CALL(inner_edge_dtrmm_nn_ru_one_12x4_lib4c)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_LIB
#else
	CALL(inner_store_4x12_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nn_ru_one_4x12_tran_lib4c4c)





//                                             1      2              3          4          5        6             7          8        9          10       11      12     13
// void kernel_dtrmm_nn_ru_one_4x12_tran_vs_lib4c4c(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_ru_one_4x12_tran_vs_lib4c4c)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

	movq	ARG12, %r15  // m1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG12, %r15  // m1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG12, %r15  // m1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif

103:

	movq	ARG12, %r15 // m1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NN_RU_ONE_12X4_VS_LIB4C
#else
	CALL(inner_edge_dtrmm_nn_ru_one_12x4_vs_lib4c)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_VS_LIB
#else
	CALL(inner_store_4x12_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nn_ru_one_4x12_tran_vs_lib4c4c)





//                                 1      2              3          4        5          6             7          8        9          10
// void kernel_dtrmm_nt_rl_12x4_lib44cc(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_rl_12x4_lib44cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RL_12X4_LIB4
#else
	CALL(inner_edge_dtrmm_nt_rl_12x4_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_rl_12x4_lib44cc)





//                                    1      2              3          4        5          6             7          8        9          10       11      12
// void kernel_dtrmm_nt_rl_12x4_vs_lib44cc(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_rl_12x4_vs_lib44cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif

	movq	ARG12, %r14 // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RL_12X4_VS_LIB4
#else
	CALL(inner_edge_dtrmm_nt_rl_12x4_vs_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG11, %r14 // m1
	movq	ARG12, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG11, %r12 // m1
	movq	ARG12, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_rl_12x4_vs_lib44cc)





//                                        1      2              3          4        5          6             7          8        9          10
// void kernel_dtrmm_nt_rl_one_12x4_lib44cc(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_rl_one_12x4_lib44cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RL_ONE_12X4_LIB4
#else
	CALL(inner_edge_dtrmm_nt_rl_one_12x4_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_rl_one_12x4_lib44cc)





//                                           1      2              3          4        5          6             7          8        9          10       11      12
// void kernel_dtrmm_nt_rl_one_12x4_vs_lib44cc(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_rl_one_12x4_vs_lib44cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif

	movq	ARG12, %r14 // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RL_ONE_12X4_VS_LIB4
#else
	CALL(inner_edge_dtrmm_nt_rl_one_12x4_vs_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG11, %r14 // m1
	movq	ARG12, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG11, %r12 // m1
	movq	ARG12, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_rl_one_12x4_vs_lib44cc)





//                                    1      2              3          4          5             6          7        8          9   10         11
// void kernel_dtrmm_nt_rl_4x12_tran_lib444c(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_rl_4x12_tran_lib444c)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif

	// final triangle

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RL_12X4_LIB4
#else
	CALL(inner_edge_dtrmm_nt_rl_12x4_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_LIB
#else
	CALL(inner_store_4x12_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_rl_4x12_tran_lib444c)





//                                             1      2              3          4          5             6          7        8          9        10       11      12
// void kernel_dtrmm_nt_rl_4x12_tran_vs_lib444c(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_rl_4x12_tran_vs_lib444c)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif

	// final triangle

	movq	ARG11, %r14 // m1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RL_12X4_VS_LIB4
#else
	CALL(inner_edge_dtrmm_nt_rl_12x4_vs_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG11, %r12   // m1
	movq	ARG12, %r13   // n1

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_VS_LIB
#else
	CALL(inner_store_4x12_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_rl_4x12_tran_vs_lib444c)





//                                    1      2              3          4          5             6          7        8          9   10         11
// void kernel_dtrmm_nt_rl_one_4x12_tran_lib444c(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_rl_one_4x12_tran_lib444c)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif

	// final triangle

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RL_ONE_12X4_LIB4
#else
	CALL(inner_edge_dtrmm_nt_rl_one_12x4_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_LIB
#else
	CALL(inner_store_4x12_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_rl_one_4x12_tran_lib444c)





//                                             1      2              3          4          5             6          7        8          9        10       11      12
// void kernel_dtrmm_nt_rl_one_4x12_tran_vs_lib444c(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_rl_one_4x12_tran_vs_lib444c)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif

	// final triangle

	movq	ARG11, %r14 // m1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RL_ONE_12X4_VS_LIB4
#else
	CALL(inner_edge_dtrmm_nt_rl_one_12x4_vs_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG11, %r12   // m1
	movq	ARG12, %r13   // n1

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_VS_LIB
#else
	CALL(inner_store_4x12_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_rl_one_4x12_tran_vs_lib444c)





//                                     1      2              3          4        5          6        7             8          9        10         11
// void kernel_dtrmm_nt_rl_12x4_lib4ccc(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd,);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_rl_12x4_lib4ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RL_12X4_LIB4C
#else
	CALL(inner_edge_dtrmm_nt_rl_12x4_lib4c)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_rl_12x4_lib4ccc)





//                                        1      2              3          4        5          6        7             8          9        10         11       12      13
// void kernel_dtrmm_nt_rl_12x4_vs_lib4ccc(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_rl_12x4_vs_lib4ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

	movq	ARG13, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG13, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG13, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif

103:

	movq	ARG13, %r15  // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RL_12X4_VS_LIB4C
#else
	CALL(inner_edge_dtrmm_nt_rl_12x4_vs_lib4c)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG12, %r14 // m1
	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_rl_12x4_vs_lib4ccc)





//                                         1      2              3          4        5          6        7             8          9        10         11
// void kernel_dtrmm_nt_rl_one_12x4_lib4ccc(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd,);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_rl_one_12x4_lib4ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RL_ONE_12X4_LIB4C
#else
	CALL(inner_edge_dtrmm_nt_rl_one_12x4_lib4c)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_rl_one_12x4_lib4ccc)





//                                            1      2              3          4        5          6        7             8          9        10         11       12      13
// void kernel_dtrmm_nt_rl_one_12x4_vs_lib4ccc(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_rl_one_12x4_vs_lib4ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

	movq	ARG13, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG13, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG13, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif

103:

	movq	ARG13, %r15  // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RL_ONE_12X4_VS_LIB4C
#else
	CALL(inner_edge_dtrmm_nt_rl_one_12x4_vs_lib4c)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG12, %r14 // m1
	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_rl_one_12x4_vs_lib4ccc)





//                                         1      2              3          4          5        6             7          8          9       10         11
// void kernel_dtrmm_nt_rl_4x12_tran_lib4c4c(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_rl_4x12_tran_lib4c4c)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RL_12X4_LIB4C
#else
	CALL(inner_edge_dtrmm_nt_rl_12x4_lib4c)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_LIB
#else
	CALL(inner_store_4x12_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_rl_4x12_tran_lib4c4c)





//                                            1      2              3          4          5        6             7          8          9        10      11         12      13
// void kernel_dtrmm_nt_rl_4x12_tran_vs_lib4c4c(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_rl_4x12_tran_vs_lib4c4c)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

	movq	ARG12, %r15  // m1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG12, %r15  // m1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG12, %r15  // m1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif

103:

	movq	ARG12, %r15 // m1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RL_12X4_VS_LIB4C
#else
	CALL(inner_edge_dtrmm_nt_rl_12x4_vs_lib4c)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_VS_LIB
#else
	CALL(inner_store_4x12_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_rl_4x12_tran_vs_lib4c4c)





//                                         1      2              3          4          5        6             7          8          9       10         11
// void kernel_dtrmm_nt_rl_one_4x12_tran_lib4c4c(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_rl_one_4x12_tran_lib4c4c)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RL_ONE_12X4_LIB4C
#else
	CALL(inner_edge_dtrmm_nt_rl_one_12x4_lib4c)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_LIB
#else
	CALL(inner_store_4x12_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_rl_one_4x12_tran_lib4c4c)





//                                            1      2              3          4          5        6             7          8          9        10      11         12      13
// void kernel_dtrmm_nt_rl_one_4x12_tran_vs_lib4c4c(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_rl_one_4x12_tran_vs_lib4c4c)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

	movq	ARG12, %r15  // m1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG12, %r15  // m1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG12, %r15  // m1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif

103:

	movq	ARG12, %r15 // m1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RL_ONE_12X4_VS_LIB4C
#else
	CALL(inner_edge_dtrmm_nt_rl_one_12x4_vs_lib4c)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_VS_LIB
#else
	CALL(inner_store_4x12_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_rl_one_4x12_tran_vs_lib4c4c)





//                                      1      2              3          4        5          6             7          8        9          10
// void kernel_dtrmm_nt_ru_12x4_lib444c(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_ru_12x4_lib444c)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
//	subl	$ 4, %r10d //k-4
	movq	ARG3, %r11  // A
//	addq	$ 128, %r11 // A+4*bs
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
//	addq	$ 128, %r13 // B+4*bs

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RU_12X4_LIB4
#else
	CALL(inner_edge_dtrmm_nt_ru_12x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender nn

#if MACRO_LEVEL>=1
//	INNER_BLEND_12X4_LIB4
#else
//	CALL(inner_blend_12x4_lib4)
#endif


	// initial triangle

//	movq	ARG1, %r10
//	movq	ARG3, %r11 // A
//	movq	ARG4, %r12 // sda
//	sall	$ 5, %r12d // 4*sda*sizeof(double)
//	movq	ARG5, %r13 // B

#if MACRO_LEVEL>=1
//	INNER_EDGE_DTRMM_NT_RU_12X4_VS_LIB4
#else
//	CALL(inner_edge_dtrmm_nt_ru_12x4_vs_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // sdc
	sall	$ 5, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_ru_12x4_lib444c)





//                                         1      2              3          4        5          6             7          8        9          10       11      12
// void kernel_dtrmm_nt_ru_12x4_vs_lib444c(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_ru_12x4_vs_lib444c)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
//	subl	$ 4, %r10d //k-4
	movq	ARG3, %r11  // A
//	addq	$ 128, %r11 // A+4*bs
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
//	addq	$ 128, %r13 // B+4*bs

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RU_12X4_VS_LIB4
#else
	CALL(inner_edge_dtrmm_nt_ru_12x4_vs_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender nn

#if MACRO_LEVEL>=1
//	INNER_BLEND_12X4_LIB4
#else
//	CALL(inner_blend_12x4_lib4)
#endif


	// initial triangle

//	movq	ARG1, %r10
//	movq	ARG3, %r11 // A
//	movq	ARG4, %r12 // sda
//	sall	$ 5, %r12d // 4*sda*sizeof(double)
//	movq	ARG5, %r13 // B

#if MACRO_LEVEL>=1
//	INNER_EDGE_DTRMM_NT_RU_12X4_VS_LIB4
#else
//	CALL(inner_edge_dtrmm_nt_ru_12x4_vs_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // sdc
	sall	$ 5, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG11, %r12 // m1
	movq	ARG12, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_ru_12x4_vs_lib444c)





//                                 1      2              3          4        5          6             7          8        9          10
// void kernel_dtrmm_nt_ru_12x4_lib44cc(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_ru_12x4_lib44cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
//	subl	$ 4, %r10d //k-4
	movq	ARG3, %r11  // A
//	addq	$ 128, %r11 // A+4*bs
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
//	addq	$ 128, %r13 // B+4*bs

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RU_12X4_LIB4
#else
	CALL(inner_edge_dtrmm_nt_ru_12x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender nn

#if MACRO_LEVEL>=1
//	INNER_BLEND_12X4_LIB4
#else
//	CALL(inner_blend_12x4_lib4)
#endif


	// initial triangle

//	movq	ARG1, %r10
//	movq	ARG3, %r11 // A
//	movq	ARG4, %r12 // sda
//	sall	$ 5, %r12d // 4*sda*sizeof(double)
//	movq	ARG5, %r13 // B

#if MACRO_LEVEL>=1
//	INNER_EDGE_DTRMM_NT_RU_12X4_VS_LIB4
#else
//	CALL(inner_edge_dtrmm_nt_ru_12x4_vs_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_ru_12x4_lib44cc)





//                                    1      2              3          4        5          6             7          8        9          10       11      12
// void kernel_dtrmm_nt_ru_12x4_vs_lib44cc(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_ru_12x4_vs_lib44cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
//	subl	$ 4, %r10d //k-4
	movq	ARG3, %r11  // A
//	addq	$ 128, %r11 // A+4*bs
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
//	addq	$ 128, %r13 // B+4*bs

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RU_12X4_VS_LIB4
#else
	CALL(inner_edge_dtrmm_nt_ru_12x4_vs_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender nn

#if MACRO_LEVEL>=1
//	INNER_BLEND_12X4_LIB4
#else
//	CALL(inner_blend_12x4_lib4)
#endif


	// initial triangle

//	movq	ARG1, %r10
//	movq	ARG3, %r11 // A
//	movq	ARG4, %r12 // sda
//	sall	$ 5, %r12d // 4*sda*sizeof(double)
//	movq	ARG5, %r13 // B

#if MACRO_LEVEL>=1
//	INNER_EDGE_DTRMM_NT_RU_12X4_VS_LIB4
#else
//	CALL(inner_edge_dtrmm_nt_ru_12x4_vs_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG11, %r14 // m1
	movq	ARG12, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG11, %r12 // m1
	movq	ARG12, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_ru_12x4_vs_lib44cc)





//                                 1      2              3          4        5          6        7             8          9        10         11
// void kernel_dtrmm_nt_ru_12x4_lib4ccc(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd,);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_ru_12x4_lib4ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RU_12X4_LIB4C
#else
	CALL(inner_edge_dtrmm_nt_ru_12x4_lib4c)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_ru_12x4_lib4ccc)





//                                    1      2              3          4        5          6        7             8          9        10         11       12      13
// void kernel_dtrmm_nt_ru_12x4_vs_lib4ccc(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_ru_12x4_vs_lib4ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RU_12X4_VS_LIB4C
#else
	CALL(inner_edge_dtrmm_nt_ru_12x4_vs_lib4c)
#endif

	movq	ARG13, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG13, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG13, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif

103:


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG12, %r14 // m1
	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_ru_12x4_vs_lib4ccc)





//                                          1      2              3          4          5        6             7          8        9
// void kernel_dtrmm_nt_ru_4x12_tran_lib4c4c(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_ru_4x12_tran_lib4c4c)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RU_12X4_LIB4C
#else
	CALL(inner_edge_dtrmm_nt_ru_12x4_lib4c)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_LIB
#else
	CALL(inner_store_4x12_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_ru_4x12_tran_lib4c4c)





//                                             1      2              3          4          5        6             7          8        9          10       11       12      13
// void kernel_dtrmm_nt_ru_4x12_tran_vs_lib4c4c(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_ru_4x12_tran_vs_lib4c4c)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RU_12X4_VS_LIB4C
#else
	CALL(inner_edge_dtrmm_nt_ru_12x4_vs_lib4c)
#endif

	movq	ARG12, %r15  // m1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG12, %r15  // m1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG12, %r15  // m1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif

103:


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_VS_LIB
#else
	CALL(inner_store_4x12_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_ru_4x12_tran_vs_lib4c4c)





//                                          1      2              3          4          5        6             7          8        9
// void kernel_dtrmm_nt_ru_one_4x12_tran_lib4c4c(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_ru_one_4x12_tran_lib4c4c)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RU_ONE_12X4_LIB4C
#else
	CALL(inner_edge_dtrmm_nt_ru_one_12x4_lib4c)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_LIB
#else
	CALL(inner_store_4x12_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_ru_one_4x12_tran_lib4c4c)





//                                             1      2              3          4          5        6             7          8        9          10       11       12      13
// void kernel_dtrmm_nt_ru_one_4x12_tran_vs_lib4c4c(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_ru_one_4x12_tran_vs_lib4c4c)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RU_ONE_12X4_VS_LIB4C
#else
	CALL(inner_edge_dtrmm_nt_ru_one_12x4_vs_lib4c)
#endif

	movq	ARG12, %r15  // m1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG12, %r15  // m1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG12, %r15  // m1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif

103:


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_VS_LIB
#else
	CALL(inner_store_4x12_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_ru_one_4x12_tran_vs_lib4c4c)





//                                         1      2              3          4        5          6             7          8        9          10
// void kernel_dtrmm_nt_ru_one_12x4_lib44cc(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_ru_one_12x4_lib44cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
//	subl	$ 4, %r10d //k-4
	movq	ARG3, %r11  // A
//	addq	$ 128, %r11 // A+4*bs
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
//	addq	$ 128, %r13 // B+4*bs

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RU_ONE_12X4_LIB4
#else
	CALL(inner_edge_dtrmm_nt_ru_one_12x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender nn

#if MACRO_LEVEL>=1
//	INNER_BLEND_12X4_LIB4
#else
//	CALL(inner_blend_12x4_lib4)
#endif


	// initial triangle

//	movq	ARG1, %r10
//	movq	ARG3, %r11 // A
//	movq	ARG4, %r12 // sda
//	sall	$ 5, %r12d // 4*sda*sizeof(double)
//	movq	ARG5, %r13 // B

#if MACRO_LEVEL>=1
//	INNER_EDGE_DTRMM_NT_RU_12X4_VS_LIB4
#else
//	CALL(inner_edge_dtrmm_nt_ru_12x4_vs_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_ru_one_12x4_lib44cc)





//                                            1      2              3          4        5          6             7          8        9          10       11      12
// void kernel_dtrmm_nt_ru_one_12x4_vs_lib44cc(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_ru_one_12x4_vs_lib44cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
//	subl	$ 4, %r10d //k-4
	movq	ARG3, %r11  // A
//	addq	$ 128, %r11 // A+4*bs
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
//	addq	$ 128, %r13 // B+4*bs

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RU_ONE_12X4_VS_LIB4
#else
	CALL(inner_edge_dtrmm_nt_ru_one_12x4_vs_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender nn

#if MACRO_LEVEL>=1
//	INNER_BLEND_12X4_LIB4
#else
//	CALL(inner_blend_12x4_lib4)
#endif


	// initial triangle

//	movq	ARG1, %r10
//	movq	ARG3, %r11 // A
//	movq	ARG4, %r12 // sda
//	sall	$ 5, %r12d // 4*sda*sizeof(double)
//	movq	ARG5, %r13 // B

#if MACRO_LEVEL>=1
//	INNER_EDGE_DTRMM_NT_RU_12X4_VS_LIB4
#else
//	CALL(inner_edge_dtrmm_nt_ru_12x4_vs_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG11, %r14 // m1
	movq	ARG12, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG11, %r12 // m1
	movq	ARG12, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_ru_one_12x4_vs_lib44cc)





//                                         1      2              3          4          5             6          7        8         9          10
// void kernel_dtrmm_nt_ru_4x12_tran_lib444c(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_ru_4x12_tran_lib444c)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13 // B

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RU_12X4_LIB4
#else
	CALL(inner_edge_dtrmm_nt_ru_12x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_LIB
#else
	CALL(inner_store_4x12_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_ru_4x12_tran_lib444c)





//                                             1      2              3          4          5             6          7        8          9        10      11       12
// void kernel_dtrmm_nt_ru_4x12_tran_vs_lib444c(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_ru_4x12_tran_vs_lib444c)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13 // B

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RU_12X4_VS_LIB4
#else
	CALL(inner_edge_dtrmm_nt_ru_12x4_vs_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG11, %r12   // m1
	movq	ARG12, %r13   // n1

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_VS_LIB
#else
	CALL(inner_store_4x12_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_ru_4x12_tran_vs_lib444c)





//                                         1      2              3          4          5             6          7        8         9          10
// void kernel_dtrmm_nt_ru_one_4x12_tran_lib444c(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_ru_one_4x12_tran_lib444c)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13 // B

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RU_ONE_12X4_LIB4
#else
	CALL(inner_edge_dtrmm_nt_ru_one_12x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_LIB
#else
	CALL(inner_store_4x12_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_ru_one_4x12_tran_lib444c)





//                                             1      2              3          4          5             6          7        8          9        10      11       12
// void kernel_dtrmm_nt_ru_one_4x12_tran_vs_lib444c(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_ru_one_4x12_tran_vs_lib444c)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13 // B

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RU_ONE_12X4_VS_LIB4
#else
	CALL(inner_edge_dtrmm_nt_ru_one_12x4_vs_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG11, %r12   // m1
	movq	ARG12, %r13   // n1

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_VS_LIB
#else
	CALL(inner_store_4x12_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_ru_one_4x12_tran_vs_lib444c)





//                                        1      2              3          4        5          6        7             8          9        10         11
// void kernel_dtrmm_nt_ru_one_12x4_lib4ccc(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd,);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_ru_one_12x4_lib4ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RU_ONE_12X4_LIB4C
#else
	CALL(inner_edge_dtrmm_nt_ru_one_12x4_lib4c)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_ru_one_12x4_lib4ccc)





//                                           1      2              3          4        5          6        7             8          9        10         11       12      13
// void kernel_dtrmm_nt_ru_one_12x4_vs_lib4ccc(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_ru_one_12x4_vs_lib4ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RU_ONE_12X4_VS_LIB4C
#else
	CALL(inner_edge_dtrmm_nt_ru_one_12x4_vs_lib4c)
#endif

	movq	ARG13, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG13, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG13, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif

103:


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG12, %r14 // m1
	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_ru_one_12x4_vs_lib4ccc)





//                                     1      2          3        4          5          6        7          8        9
// void kernel_dpotrf_nt_l_12x4_lib44cc(int k, double *A, int sda, double *B, double *C, int ldc, double *D, int ldd, double *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpotrf_nt_l_12x4_lib44cc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // ldb
	sall	$ 5, %r12d
	movq	ARG4, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // C
	movq	ARG6, %r11 // ldc
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB
#else
	CALL(inner_scale_m11_12x4_lib)
#endif


	// factorization

	movq	ARG9, %r10  // inv_diag_D 
	movl	$ 4, %r11d

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_12X4_VS_LIB4
#else
	CALL(inner_edge_dpotrf_12x4_vs_lib4)
#endif


	// store

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_L_12X4_LIB
#else
	CALL(inner_store_l_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dpotrf_nt_l_12x4_lib44cc)





//                                        1      2          3        4          5          6        7          8        9                   10      11
// void kernel_dpotrf_nt_l_12x4_vs_lib44cc(int k, double *A, int sda, double *B, double *C, int ldc, double *D, int ldd, double *inv_diag_D, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpotrf_nt_l_12x4_vs_lib44cc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // ldb
	sall	$ 5, %r12d
	movq	ARG4, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // C
	movq	ARG6, %r11 // ldc
	sall	$ 3, %r11d
	movq	ARG10, %r12 // m1
	movq	ARG11, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_VS_LIB
#else
	CALL(inner_scale_m11_12x4_vs_lib)
#endif


	// factorization

	movq	ARG9, %r10  // inv_diag_D 
	movq	ARG11, %r11 // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_12X4_VS_LIB4
#else
	CALL(inner_edge_dpotrf_12x4_vs_lib4)
#endif


	// store

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG10, %r12 // m1
	movq	ARG11, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_12X4_VS_LIB
#else
	CALL(inner_store_l_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dpotrf_nt_l_12x4_vs_lib44cc)





//                                          1      2          3        4          5        6             7          8        9          10       11         12       13
// void kernel_dtrsm_nn_rl_inv_12x4_lib4c44c(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int sdd, double *E, int lde, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_rl_inv_12x4_lib4c44c)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // sdc
	sall	$ 5, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d
	movq	ARG13, %r12  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLN_INV_12X4_LIB
#else
	CALL(inner_edge_dtrsm_rln_inv_12x4_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_rl_inv_12x4_lib4c44c)





//                                             1      2          3        4          5        6             7          8        9          10       11         12       13                  14      15
// void kernel_dtrsm_nn_rl_inv_12x4_vs_lib4c44c(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int sdd, double *E, int lde, double *inv_diag_E, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_rl_inv_12x4_vs_lib4c44c)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

	movq	ARG15, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG15, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG15, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif

103:


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // sdc
	sall	$ 5, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d
	movq	ARG13, %r12  // inv_diag_E 
	movq	ARG15, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLN_INV_12X4_VS_LIB
#else
	CALL(inner_edge_dtrsm_rln_inv_12x4_vs_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d
	movq	ARG14, %r12 // m1
	movq	ARG15, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_rl_inv_12x4_vs_lib4c44c)





//                                          1      2          3        4          5        6             7          8        9          10       11         12       13
// void kernel_dtrsm_nn_rl_inv_12x4_lib4cccc(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_rl_inv_12x4_lib4cccc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d
	movq	ARG4, %r13
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // ldc
	sall	$ 3, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB
#else
	CALL(inner_scale_m1b_12x4_lib)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d
	movq	ARG13, %r12  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLN_INV_12X4_LIB
#else
	CALL(inner_edge_dtrsm_rln_inv_12x4_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_rl_inv_12x4_lib4cccc)





//                                            1      2          3        4          5        6              7          8        9          10       11         12       13                  14      15
// void kernel_dtrsm_nn_rl_inv_12x4_vs_lib4cccc(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, double *inv_diag_E, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_rl_inv_12x4_vs_lib4cccc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d
	movq	ARG4, %r13
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

	movq	ARG15, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG15, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG15, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif

103:


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // ldc
	sall	$ 3, %r12d
	movq	ARG14, %r13 // m1
	movq	ARG15, %r14 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_VS_LIB
#else
	CALL(inner_scale_m1b_12x4_vs_lib)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d
	movq	ARG13, %r12  // inv_diag_E 
	movq	ARG15, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLN_INV_12X4_VS_LIB
#else
	CALL(inner_edge_dtrsm_rln_inv_12x4_vs_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG14, %r12 // m1
	movq	ARG15, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_rl_inv_12x4_vs_lib4cccc)





//                                          1      2          3        4          5        6             7          8        9          10       11         12
// void kernel_dtrsm_nn_rl_one_12x4_lib4c44c(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int sdd, double *E, int lde);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_rl_one_12x4_lib4c44c)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // sdc
	sall	$ 5, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLN_ONE_12X4_LIB
#else
	CALL(inner_edge_dtrsm_rln_one_12x4_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_rl_one_12x4_lib4c44c)





//                                             1      2          3        4          5        6             7          8        9          10       11         12       13      14
// void kernel_dtrsm_nn_rl_one_12x4_vs_lib4c44c(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int sdd, double *E, int lde, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_rl_one_12x4_vs_lib4c44c)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

	movq	ARG14, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG14, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG14, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif

103:


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // sdc
	sall	$ 5, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d
	movq	ARG14, %r12 // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLN_ONE_12X4_VS_LIB
#else
	CALL(inner_edge_dtrsm_rln_one_12x4_vs_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d
	movq	ARG13, %r12 // m1
	movq	ARG14, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_rl_one_12x4_vs_lib4c44c)





//                                          1      2          3        4          5        6             7          8        9          10       11         12
// void kernel_dtrsm_nn_rl_one_12x4_lib4cccc(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_rl_one_12x4_lib4cccc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d
	movq	ARG4, %r13
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // ldc
	sall	$ 3, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB
#else
	CALL(inner_scale_m1b_12x4_lib)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLN_ONE_12X4_LIB
#else
	CALL(inner_edge_dtrsm_rln_one_12x4_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_rl_one_12x4_lib4cccc)





//                                             1      2          3        4          5        6             7          8        9          10       11         12       13      14
// void kernel_dtrsm_nn_rl_one_12x4_vs_lib4cccc(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_rl_one_12x4_vs_lib4cccc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d
	movq	ARG4, %r13
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

	movq	ARG14, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG14, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG14, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif

103:


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // ldc
	sall	$ 3, %r12d
	movq	ARG13, %r13 // m1
	movq	ARG14, %r14 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_VS_LIB
#else
	CALL(inner_scale_m1b_12x4_vs_lib)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d
	movq	ARG14, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLN_ONE_12X4_VS_LIB
#else
	CALL(inner_edge_dtrsm_rln_one_12x4_vs_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG13, %r12 // m1
	movq	ARG14, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_rl_one_12x4_vs_lib4cccc)





//                                          1      2          3        4          5             6          7        8          9        10         11
// void kernel_dtrsm_nt_rl_inv_12x4_lib44cc4(int k, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_inv_12x4_lib44cc4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d
	movq	ARG4, %r13

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C
	movq	ARG7, %r12 // ldc
	sall	$ 3, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB
#else
	CALL(inner_scale_m1b_12x4_lib)
#endif


	// solve

	movq	ARG10, %r10  // E 
	movq	ARG11, %r11  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_12X4_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_inv_12x4_lib4)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_12x4_lib44cc4)





//                                             1      2          3        4          5             6          7        8          9        10         11                  12      13
// void kernel_dtrsm_nt_rl_inv_12x4_vs_lib44cc4(int k, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E, double *inv_diag_E, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_inv_12x4_vs_lib44cc4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d
	movq	ARG4, %r13

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C
	movq	ARG7, %r12 // ldc
	sall	$ 3, %r12d
	movq	ARG12, %r13 // m1
	movq	ARG13, %r14 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_VS_LIB
#else
	CALL(inner_scale_m1b_12x4_vs_lib)
#endif


	// solve

	movq	ARG10, %r10  // E 
	movq	ARG11, %r11  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_12X4_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_inv_12x4_lib4)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_12x4_vs_lib44cc4)





//                                         1      2          3        4          5          6        7          8        9          10       11
// void kernel_dtrsm_nt_rl_inv_12x4_lib44ccc(int k, double *A, int sda, double *B, double *C, int ldc, double *D, int ldd, double *E, int lde, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_inv_12x4_lib44ccc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d
	movq	ARG4, %r13

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // C
	movq	ARG6, %r11 // ldc
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB
#else
	CALL(inner_scale_m11_12x4_lib)
#endif


	// solve

	movq	ARG9, %r10  // E 
	movq	ARG10, %r11 // lde
	sall	$ 3, %r11d
	movq	ARG11, %r12  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_12X4_LIB
#else
	CALL(inner_edge_dtrsm_rlt_inv_12x4_lib)
#endif


	// store

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_12x4_lib44ccc)





//                                           1      2          3        4          5          6        7          8        9          10       11                 12      13
// void kernel_dtrsm_nt_rl_inv_12x4_vs_lib44ccc(int k, double *A, int sda, double *B, double *C, int ldc, double *D, int ldd, double *E, int lde, double *inv_diag_E, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_inv_12x4_vs_lib44ccc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d
	movq	ARG4, %r13

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // C
	movq	ARG6, %r11 // ldc
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_VS_LIB
#else
	CALL(inner_scale_m11_12x4_vs_lib)
#endif


	// solve

	movq	ARG9, %r10  // E 
	movq	ARG10, %r11 // lde
	sall	$ 3, %r11d
	movq	ARG11, %r12  // inv_diag_E 
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_12X4_VS_LIB
#else
	CALL(inner_edge_dtrsm_rlt_inv_12x4_vs_lib)
#endif


	// store

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_12x4_vs_lib44ccc)





//                                          1      2          3        4          5        6             7          8        9          10       11         12       13
// void kernel_dtrsm_nt_rl_inv_12x4_lib4c44c(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int sdd, double *E, int lde, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_inv_12x4_lib4c44c)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d
	movq	ARG4, %r13
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // ldc
	sall	$ 5, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d
	movq	ARG13, %r12  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_12X4_LIB
#else
	CALL(inner_edge_dtrsm_rlt_inv_12x4_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 5, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_12x4_lib4c44c)





//                                             1      2          3        4          5        6             7          8        9          10       11         12       13                  14      15
// void kernel_dtrsm_nt_rl_inv_12x4_vs_lib4c44c(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int sdd, double *E, int lde, double *inv_diag_E, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_inv_12x4_vs_lib4c44c)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d
	movq	ARG4, %r13
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

	movq	ARG15, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG15, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG15, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif

103:


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // ldc
	sall	$ 5, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d
	movq	ARG13, %r12  // inv_diag_E 
	movq	ARG15, %r13  // n1 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_12X4_VS_LIB
#else
	CALL(inner_edge_dtrsm_rlt_inv_12x4_vs_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 5, %r11d
	movq	ARG14, %r12  // m1 
	movq	ARG15, %r13  // n1 

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_12x4_vs_lib4c44c)





//                                          1      2          3        4          5        6             7          8        9          10       11         12       13
// void kernel_dtrsm_nt_rl_inv_12x4_lib4cccc(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_inv_12x4_lib4cccc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d
	movq	ARG4, %r13
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // ldc
	sall	$ 3, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB
#else
	CALL(inner_scale_m1b_12x4_lib)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d
	movq	ARG13, %r12  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_12X4_LIB
#else
	CALL(inner_edge_dtrsm_rlt_inv_12x4_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_12x4_lib4cccc)





//                                            1      2          3        4          5        6              7          8        9          10       11         12       13                  14      15
// void kernel_dtrsm_nt_rl_inv_12x4_vs_lib4cccc(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, double *inv_diag_E, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_inv_12x4_vs_lib4cccc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d
	movq	ARG4, %r13
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

	movq	ARG15, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG15, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG15, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif

103:


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // ldc
	sall	$ 3, %r12d
	movq	ARG14, %r13 // m1
	movq	ARG15, %r14 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_VS_LIB
#else
	CALL(inner_scale_m1b_12x4_vs_lib)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d
	movq	ARG13, %r12  // inv_diag_E 
	movq	ARG15, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_12X4_VS_LIB
#else
	CALL(inner_edge_dtrsm_rlt_inv_12x4_vs_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG14, %r12 // m1
	movq	ARG15, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_12x4_vs_lib4cccc)





//                                          1      2          3        4          5             6          7        8          9        10
// void kernel_dtrsm_nt_rl_one_12x4_lib44cc4(int k, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_one_12x4_lib44cc4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d
	movq	ARG4, %r13

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C
	movq	ARG7, %r12 // ldc
	sall	$ 3, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB
#else
	CALL(inner_scale_m1b_12x4_lib)
#endif


	// solve

	movq	ARG10, %r10  // E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_ONE_12X4_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_one_12x4_lib4)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_rl_one_12x4_lib44cc4)





//                                             1      2          3        4          5             6          7        8          9        10         11      12
// void kernel_dtrsm_nt_rl_one_12x4_vs_lib44cc4(int k, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_one_12x4_vs_lib44cc4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d
	movq	ARG4, %r13

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C
	movq	ARG7, %r12 // ldc
	sall	$ 3, %r12d
	movq	ARG11, %r13 // m1
	movq	ARG12, %r14 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_VS_LIB
#else
	CALL(inner_scale_m1b_12x4_vs_lib)
#endif


	// solve

	movq	ARG10, %r10  // E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_ONE_12X4_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_one_12x4_lib4)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG11, %r12 // m1
	movq	ARG12, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_rl_one_12x4_vs_lib44cc4)





//                                          1      2          3        4          5        6             7          8        9          10       11         12
// void kernel_dtrsm_nt_rl_one_12x4_lib4c44c(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int sdd, double *E, int lde);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_one_12x4_lib4c44c)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d
	movq	ARG4, %r13
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // ldc
	sall	$ 5, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_ONE_12X4_LIB
#else
	CALL(inner_edge_dtrsm_rlt_one_12x4_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 5, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_rl_one_12x4_lib4c44c)





//                                             1      2          3        4          5        6             7          8        9          10       11         12       13      14
// void kernel_dtrsm_nt_rl_one_12x4_vs_lib4c44c(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int sdd, double *E, int lde, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_one_12x4_vs_lib4c44c)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d
	movq	ARG4, %r13
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

	movq	ARG14, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG14, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG14, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif

103:


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // ldc
	sall	$ 5, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d
	movq	ARG14, %r12  // n1 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_ONE_12X4_VS_LIB
#else
	CALL(inner_edge_dtrsm_rlt_one_12x4_vs_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 5, %r11d
	movq	ARG13, %r12  // m1 
	movq	ARG14, %r13  // n1 

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_rl_one_12x4_vs_lib4c44c)





//                                          1      2          3        4          5        6             7          8        9          10       11         12
// void kernel_dtrsm_nt_rl_one_12x4_lib4cccc(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_one_12x4_lib4cccc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d
	movq	ARG4, %r13
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // ldc
	sall	$ 3, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB
#else
	CALL(inner_scale_m1b_12x4_lib)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_ONE_12X4_LIB
#else
	CALL(inner_edge_dtrsm_rlt_one_12x4_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_rl_one_12x4_lib4cccc)





//                                             1      2          3        4          5        6             7          8        9          10       11         12       13      14
// void kernel_dtrsm_nt_rl_one_12x4_vs_lib4cccc(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_one_12x4_vs_lib4cccc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d
	movq	ARG4, %r13
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

	movq	ARG14, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG14, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG14, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif

103:


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // ldc
	sall	$ 3, %r12d
	movq	ARG13, %r13 // m1
	movq	ARG14, %r14 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_VS_LIB
#else
	CALL(inner_scale_m1b_12x4_vs_lib)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d
	movq	ARG14, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_ONE_12X4_VS_LIB
#else
	CALL(inner_edge_dtrsm_rlt_one_12x4_vs_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG13, %r12 // m1
	movq	ARG14, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_rl_one_12x4_vs_lib4cccc)





//                                          1      2          3        4          5        6             7          8        9          10       11         12       13
// void kernel_dtrsm_nn_ru_inv_12x4_lib4c44c(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int sdd, double *E, int lde, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ru_inv_12x4_lib4c44c)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC

	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // sdc
	sall	$ 5, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d
	movq	ARG13, %r12  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUN_INV_12X4_LIB
#else
	CALL(inner_edge_dtrsm_run_inv_12x4_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_ru_inv_12x4_lib4c44c)





//                                             1      2          3        4          5        6             7          8        9          10       11         12       13                  14      15
// void kernel_dtrsm_nn_ru_inv_12x4_vs_lib4c44c(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int sdd, double *E, int lde, double *inv_diag_E, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ru_inv_12x4_vs_lib4c44c)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

	movq	ARG15, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG15, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG15, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif

103:


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // sdc
	sall	$ 5, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d
	movq	ARG13, %r12  // inv_diag_E 
	movq	ARG15, %r13  // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUN_INV_12X4_VS_LIB
#else
	CALL(inner_edge_dtrsm_run_inv_12x4_vs_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d
	movq	ARG14, %r12  // m1
	movq	ARG15, %r13  // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_ru_inv_12x4_vs_lib4c44c)





//                                          1      2          3        4          5        6             7          8        9          10       11         12       13
// void kernel_dtrsm_nn_ru_inv_12x4_lib4cccc(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ru_inv_12x4_lib4cccc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // ldc
	sall	$ 3, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB
#else
	CALL(inner_scale_m1b_12x4_lib)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d
	movq	ARG13, %r12  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUN_INV_12X4_LIB
#else
	CALL(inner_edge_dtrsm_run_inv_12x4_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_ru_inv_12x4_lib4cccc)





//                                            1      2          3        4          5        6             7          8        9          10       11         12       13                  14      15
// void kernel_dtrsm_nn_ru_inv_12x4_vs_lib4cccc(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, double *inv_diag_E, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ru_inv_12x4_vs_lib4cccc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

	movq	ARG15, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG15, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG15, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif

103:


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // ldc
	sall	$ 3, %r12d
	movq	ARG14, %r13  // m1
	movq	ARG15, %r14  // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_VS_LIB
#else
	CALL(inner_scale_m1b_12x4_vs_lib)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d
	movq	ARG13, %r12  // inv_diag_E 
	movq	ARG15, %r13  // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUN_INV_12X4_VS_LIB
#else
	CALL(inner_edge_dtrsm_run_inv_12x4_vs_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG14, %r12  // m1
	movq	ARG15, %r13  // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_ru_inv_12x4_vs_lib4cccc)




//                                          1      2          3        4          5        6             7          8        9          10       11         12
// void kernel_dtrsm_nn_ru_one_12x4_lib4c44c(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int sdd, double *E, int lde);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ru_one_12x4_lib4c44c)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // sdc
	sall	$ 5, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUN_ONE_12X4_LIB
#else
	CALL(inner_edge_dtrsm_run_one_12x4_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_ru_one_12x4_lib4c44c)





//                                             1      2          3        4          5        6             7          8        9          10       11         12       13      14
// void kernel_dtrsm_nn_ru_one_12x4_vs_lib4c44c(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int sdd, double *E, int lde, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ru_one_12x4_vs_lib4c44c)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

	movq	ARG14, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG14, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG14, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif

103:


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // sdc
	sall	$ 5, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d
	movq	ARG14, %r12  // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUN_ONE_12X4_VS_LIB
#else
	CALL(inner_edge_dtrsm_run_one_12x4_vs_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d
	movq	ARG13, %r12  // m1
	movq	ARG14, %r13  // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_ru_one_12x4_vs_lib4c44c)





//                                          1      2          3        4          5        6             7          8        9          10       11         12
// void kernel_dtrsm_nn_ru_one_12x4_lib4cccc(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ru_one_12x4_lib4cccc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // ldc
	sall	$ 3, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB
#else
	CALL(inner_scale_m1b_12x4_lib)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUN_ONE_12X4_LIB
#else
	CALL(inner_edge_dtrsm_run_one_12x4_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_ru_one_12x4_lib4cccc)





//                                             1      2          3        4          5        6             7          8        9          10       11         12       13      14
// void kernel_dtrsm_nn_ru_one_12x4_vs_lib4cccc(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ru_one_12x4_vs_lib4cccc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

	movq	ARG14, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG14, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG14, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif

103:


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // ldc
	sall	$ 3, %r12d
	movq	ARG13, %r13  // m1
	movq	ARG14, %r14  // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_VS_LIB
#else
	CALL(inner_scale_m1b_12x4_vs_lib)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d
	movq	ARG14, %r12  // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUN_ONE_12X4_VS_LIB
#else
	CALL(inner_edge_dtrsm_run_one_12x4_vs_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG13, %r12  // m1
	movq	ARG14, %r13  // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_ru_one_12x4_vs_lib4cccc)




//                                          1      2          3        4          5             6          7        8          9        10         11
// void kernel_dtrsm_nt_ru_inv_12x4_lib44cc4(int k, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_ru_inv_12x4_lib44cc4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d
	movq	ARG4, %r13

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C
	movq	ARG7, %r12 // ldc
	sall	$ 3, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB
#else
	CALL(inner_scale_m1b_12x4_lib)
#endif


	// solve

	movq	ARG10, %r10  // E 
	movq	ARG11, %r11  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUT_INV_12X4_LIB4
#else
	CALL(inner_edge_dtrsm_rut_inv_12x4_lib4)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_ru_inv_12x4_lib44cc4)





//                                             1      2          3        4          5             6          7        8          9        10         11                  12      13
// void kernel_dtrsm_nt_ru_inv_12x4_vs_lib44cc4(int k, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E, double *inv_diag_E, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_ru_inv_12x4_vs_lib44cc4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d
	movq	ARG4, %r13

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C
	movq	ARG7, %r12 // ldc
	sall	$ 3, %r12d
	movq	ARG12, %r13 // m1
	movq	ARG13, %r14 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_VS_LIB
#else
	CALL(inner_scale_m1b_12x4_vs_lib)
#endif


	// solve

	movq	ARG10, %r10  // E 
	movq	ARG11, %r11  // inv_diag_E 
	movq	ARG13, %r12 // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUT_INV_12X4_VS_LIB4
#else
	CALL(inner_edge_dtrsm_rut_inv_12x4_vs_lib4)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_ru_inv_12x4_vs_lib44cc4)





//                                          1      2          3        4          5        6             7          8        9          10       11         12       13
// void kernel_dtrsm_nt_ru_inv_12x4_lib4c44c(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int sdd, double *E, int lde, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_ru_inv_12x4_lib4c44c)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // sdc
	sall	$ 5, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d
	movq	ARG13, %r12  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUT_INV_12X4_LIB
#else
	CALL(inner_edge_dtrsm_rut_inv_12x4_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_ru_inv_12x4_lib4c44c)





//                                             1      2          3        4          5        6             7          8        9          10       11         12       13                  14      15
// void kernel_dtrsm_nt_ru_inv_12x4_vs_lib4c44c(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int sdd, double *E, int lde, double *inv_diag_E, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_ru_inv_12x4_vs_lib4c44c)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

	movq	ARG15, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG15, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG15, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif

103:


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // sdc
	sall	$ 5, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d
	movq	ARG13, %r12  // inv_diag_E 
	movq	ARG15, %r13  // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUT_INV_12X4_VS_LIB
#else
	CALL(inner_edge_dtrsm_rut_inv_12x4_vs_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d
	movq	ARG14, %r12  // m1
	movq	ARG15, %r13  // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_ru_inv_12x4_vs_lib4c44c)




//                                          1      2          3        4          5        6             7          8        9          10       11         12       13
// void kernel_dtrsm_nt_ru_inv_12x4_lib4cccc(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_ru_inv_12x4_lib4cccc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // ldc
	sall	$ 3, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB
#else
	CALL(inner_scale_m1b_12x4_lib)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d
	movq	ARG13, %r12  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUT_INV_12X4_LIB
#else
	CALL(inner_edge_dtrsm_rut_inv_12x4_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_ru_inv_12x4_lib4cccc)





//                                             1      2          3        4          5        6             7          8        9          10       11         12       13                  14      15
// void kernel_dtrsm_nt_ru_inv_12x4_vs_lib4cccc(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, double *inv_diag_E, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_ru_inv_12x4_vs_lib4cccc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

	movq	ARG15, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG15, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG15, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif

103:


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // ldc
	sall	$ 3, %r12d
	movq	ARG14, %r13  // m1
	movq	ARG15, %r14  // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_VS_LIB
#else
	CALL(inner_scale_m1b_12x4_vs_lib)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d
	movq	ARG13, %r12  // inv_diag_E 
	movq	ARG15, %r13  // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUT_INV_12X4_VS_LIB
#else
	CALL(inner_edge_dtrsm_rut_inv_12x4_vs_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG14, %r12  // m1
	movq	ARG15, %r13  // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_ru_inv_12x4_vs_lib4cccc)




//                                          1      2          3        4          5             6          7        8          9        10
// void kernel_dtrsm_nt_ru_one_12x4_lib44cc4(int k, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_ru_one_12x4_lib44cc4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d
	movq	ARG4, %r13

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C
	movq	ARG7, %r12 // ldc
	sall	$ 3, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB
#else
	CALL(inner_scale_m1b_12x4_lib)
#endif


	// solve

	movq	ARG10, %r10  // E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUT_ONE_12X4_LIB4
#else
	CALL(inner_edge_dtrsm_rut_one_12x4_lib4)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_ru_one_12x4_lib44cc4)





//                                              1      2          3        4          5             6          7        8          9        10         11      12
// void kernel_dtrsm_nt_ru_one_12x4_vs_lib44cc4(int k, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_ru_one_12x4_vs_lib44cc4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d
	movq	ARG4, %r13

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C
	movq	ARG7, %r12 // ldc
	sall	$ 3, %r12d
	movq	ARG11, %r13 // m1
	movq	ARG12, %r14 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_VS_LIB
#else
	CALL(inner_scale_m1b_12x4_vs_lib)
#endif


	// solve

	movq	ARG10, %r10  // E 
	movq	ARG12, %r11 // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUT_ONE_12X4_VS_LIB4
#else
	CALL(inner_edge_dtrsm_rut_one_12x4_vs_lib4)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG11, %r12 // m1
	movq	ARG12, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_ru_one_12x4_vs_lib44cc4)





//                                          1      2          3        4          5        6             7          8        9          10       11         12
// void kernel_dtrsm_nt_ru_one_12x4_lib4c44c(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int sdd, double *E, int lde);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_ru_one_12x4_lib4c44c)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // sdc
	sall	$ 5, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUT_ONE_12X4_LIB
#else
	CALL(inner_edge_dtrsm_rut_one_12x4_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_ru_one_12x4_lib4c44c)





//                                             1      2          3        4          5        6             7          8        9          10       11         12       13      14
// void kernel_dtrsm_nt_ru_one_12x4_vs_lib4c44c(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int sdc, double *D, int sdd, double *E, int lde, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_ru_one_12x4_vs_lib4c44c)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

	movq	ARG14, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG14, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG14, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif

103:


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // sdc
	sall	$ 5, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d
	movq	ARG14, %r12  // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUT_ONE_12X4_VS_LIB
#else
	CALL(inner_edge_dtrsm_rut_one_12x4_vs_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d
	movq	ARG13, %r12  // m1
	movq	ARG14, %r13  // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_ru_one_12x4_vs_lib4c44c)





//                                          1      2          3        4          5        6             7          8        9          10       11         12       13
// void kernel_dtrsm_nt_ru_one_12x4_lib4cccc(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_ru_one_12x4_lib4cccc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // ldc
	sall	$ 3, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB
#else
	CALL(inner_scale_m1b_12x4_lib)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUT_ONE_12X4_LIB
#else
	CALL(inner_edge_dtrsm_rut_one_12x4_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_ru_one_12x4_lib4cccc)





//                                             1      2          3        4          5        6             7          8        9          10       11         12       13      14
// void kernel_dtrsm_nt_ru_one_12x4_vs_lib4cccc(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_ru_one_12x4_vs_lib4cccc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

	movq	ARG14, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG14, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG14, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif

103:


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // ldc
	sall	$ 3, %r12d
	movq	ARG13, %r13  // m1
	movq	ARG14, %r14  // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_VS_LIB
#else
	CALL(inner_scale_m1b_12x4_vs_lib)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d
	movq	ARG14, %r13  // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUT_ONE_12X4_VS_LIB
#else
	CALL(inner_edge_dtrsm_rut_one_12x4_vs_lib)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG13, %r12  // m1
	movq	ARG14, %r13  // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_ru_one_12x4_vs_lib4cccc)





//                                           1      2          3        4          5             6          7        8          9        10         11       12
// void kernel_dtrsm_nt_ll_inv_12x4_lib44cc4(int k, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E, int sde, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_ll_inv_12x4_lib44cc4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C
	movq	ARG7, %r12 // ldc
	sall	$ 3, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB
#else
	CALL(inner_scale_m1b_12x4_lib)
#endif


	// solve

	movq	ARG10, %r10  // E 
	movq	ARG11, %r11 // sde
	sall	$ 5, %r11d // 4*sde*sizeof(double)
	movq	ARG12, %r12  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_LLN_INV_12X4_LIB4
#else
	CALL(inner_edge_dtrsm_lln_inv_12x4_lib4)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_ll_inv_12x4_lib44cc4)





//                                              1      2          3        4          5             6          7        8          9        10         11       12                  13      14
// void kernel_dtrsm_nt_ll_inv_12x4_vs_lib44cc4(int k, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, double *inv_diag_E, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_ll_inv_12x4_vs_lib44cc4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C
	movq	ARG7, %r12 // ldc
	sall	$ 3, %r12d
	movq	ARG13, %r13 // m1
	movq	ARG14, %r14 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_VS_LIB
#else
	CALL(inner_scale_m1b_12x4_vs_lib)
#endif


	// solve

	movq	ARG10, %r10  // E 
	movq	ARG11, %r11 // sde
	sall	$ 5, %r11d // 4*sde*sizeof(double)
	movq	ARG12, %r12  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_LLN_INV_12X4_LIB4
#else
	CALL(inner_edge_dtrsm_lln_inv_12x4_lib4)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG13, %r12 // m1
	movq	ARG14, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_ll_inv_12x4_vs_lib44cc4)





//                                          1      2          3        4          5        6             7          8        9          10       11         12       13
// void kernel_dtrsm_nn_ll_inv_12x4_lib4ccc4(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, double *E, int sde, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ll_inv_12x4_lib4ccc4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d
	movq	ARG4, %r13
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // ldc
	sall	$ 3, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB
#else
	CALL(inner_scale_m1b_12x4_lib)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // sde
	sall	$ 5, %r11d // 4*sde*sizeof(double)
	movq	ARG13, %r12  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_LLN_INV_12X4_LIB4
#else
	CALL(inner_edge_dtrsm_lln_inv_12x4_lib4)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_ll_inv_12x4_lib4ccc4)





//                                            1      2          3        4          5        6             7          8        9          10       11         12       13                   14      15
// void kernel_dtrsm_nn_ll_inv_12x4_vs_lib4ccc4(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, double *inv_diag_E, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ll_inv_12x4_vs_lib4ccc4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d
	movq	ARG4, %r13
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

	movq	ARG15, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG15, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG15, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif

103:


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // ldc
	sall	$ 3, %r12d
	movq	ARG14, %r13 // m1
	movq	ARG15, %r14 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_VS_LIB
#else
	CALL(inner_scale_m1b_12x4_vs_lib)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // sde
	sall	$ 5, %r11d // 4*sde*sizeof(double)
	movq	ARG13, %r12  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_LLN_INV_12X4_LIB4
#else
	CALL(inner_edge_dtrsm_lln_inv_12x4_lib4)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG14, %r12 // m1
	movq	ARG15, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_ll_inv_12x4_vs_lib4ccc4)





//                                          1      2          3        4          5        6             7          8        9          10       11         12
// void kernel_dtrsm_nn_ll_one_12x4_lib4cccc(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ll_one_12x4_lib4cccc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d
	movq	ARG4, %r13
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // ldc
	sall	$ 3, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB
#else
	CALL(inner_scale_m1b_12x4_lib)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_LLN_ONE_12X4_LIBC
#else
	CALL(inner_edge_dtrsm_lln_one_12x4_libc)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_ll_one_12x4_lib4cccc)





//                                             1      2          3        4          5        6             7          8        9          10       11         12       13      14
// void kernel_dtrsm_nn_ll_one_12x4_vs_lib4cccc(int k, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ll_one_12x4_vs_lib4cccc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d
	movq	ARG4, %r13
	movq	ARG5, %r14 // ldb
	sall	$ 3, %r14d

	movq	ARG14, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG14, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG14, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif

103:


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // ldc
	sall	$ 3, %r12d
	movq	ARG13, %r13 // m1
	movq	ARG14, %r14 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_VS_LIB
#else
	CALL(inner_scale_m1b_12x4_vs_lib)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // lde
	sall	$ 3, %r11d
	movq	ARG13, %r12 // m1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_LLN_ONE_12X4_VS_LIBC
#else
	CALL(inner_edge_dtrsm_lln_one_12x4_vs_libc)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG13, %r12 // m1
	movq	ARG14, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_ll_one_12x4_vs_lib4cccc)





//                                 1      2              3          4          5        6             7          8        9          10
// void kernel_dgemm_nt_4x12_lib4ccc(int k, double *alpha, double *A, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_4x12_lib4ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

#if 1

	movq	ARG1, %r10 // k
	movq	ARG4, %r11  // B
	movq	ARG5, %r12  // ldb
	sall	$ 3, %r12d
	movq	ARG3, %r13  // A

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIBC4
#else
	CALL(inner_kernel_dgemm_nt_12x4_libc4)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif

#else

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B
	movq	ARG5, %r13  // ldb
	sall	$ 3, %r13d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X12_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_4x12_lib4c)
#endif

#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X12_LIB
#else
	CALL(inner_scale_ab_4x12_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_LIB
#else
	CALL(inner_store_4x12_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_4x12_lib4ccc)





//                                    1      2              3          4          5        6             7          8        9          10       11      12
// void kernel_dgemm_nt_4x12_vs_lib4ccc(int k, double *alpha, double *A, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_4x12_vs_lib4ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

#if 1

	movq	ARG1, %r10 // k
	movq	ARG4, %r11  // B
	movq	ARG5, %r12  // ldb
	sall	$ 3, %r12d
	movq	ARG3, %r13  // A

	movq	ARG11, %r14  // m1
	cmpl	$ 1, %r14d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X1_LIBC4
#else
	CALL(inner_kernel_dgemm_nt_12x1_libc4)
#endif
	
	jmp		103f

100:

	movq	ARG11, %r14  // m1
	cmpl	$ 2, %r14d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X2_LIBC4
#else
	CALL(inner_kernel_dgemm_nt_12x2_libc4)
#endif

	jmp		103f

101:

	movq	ARG11, %r14  // m1
	cmpl	$ 3, %r14d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X3_LIBC4
#else
	CALL(inner_kernel_dgemm_nt_12x3_libc4)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIBC4
#else
	CALL(inner_kernel_dgemm_nt_12x4_libc4)
#endif

103:

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif

#else

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B
	movq	ARG5, %r13  // ldb
	sall	$ 3, %r13d


	movq	ARG12, %r14  // n1
	cmpl	$ 9, %r14d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X9_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_4x9_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG12, %r14  // n1
	cmpl	$ 10, %r14d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X10_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_4x10_lib4c)
#endif

	jmp		103f

101:

	movq	ARG12, %r14  // n1
	cmpl	$ 11, %r14d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X11_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_4x11_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X12_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_4x12_lib4c)
#endif

103:

#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG11, %r14 // m1
	movq	ARG12, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X12_VS_LIB
#else
	CALL(inner_scale_ab_4x12_vs_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG11, %r12 // m1
	movq	ARG12, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_VS_LIB
#else
	CALL(inner_store_4x12_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_4x12_vs_lib4ccc)





//                                  1      2              3          4        5          6        7             8          9        10         11
// void kernel_dgemm_nt_4x12_libc4cc(int k, double *alpha, double *A, int lda, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_4x12_libc4cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// prefetch C

	movq		ARG7, %r10 // beta
	vmovsd		0(%r10), %xmm14
	vxorpd		%xmm15, %xmm15, %xmm15 // 0.0
	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			100f // end

	movq	ARG8, %r10 // C
	movq	ARG9, %r11 // ldc
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_4X12_LIB
#else
	CALL(inner_prefetch0_4x12_lib)
#endif

100:


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG5, %r11  // B
	movq	ARG6, %r12  // sdb
	sall	$ 5, %r12d
	movq	ARG3, %r13  // A
	movq	ARG4, %r14  // lda
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif


	// prefetch D

//	movq	ARG10, %r10 // D
//	movq	ARG11, %r11 // ldd
//	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
//	INNER_PREFETCH0_4X12_LIB
#else
//	CALL(inner_prefetch0_4x12_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif


#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X12_LIB
#else
	CALL(inner_scale_ab_4x12_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_LIB
#else
	CALL(inner_store_4x12_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_4x12_libc4cc)





//                                     1      2              3          4        5          6        7             8          9        10         11       12      13
// void kernel_dgemm_nt_4x12_vs_libc4cc(int k, double *alpha, double *A, int lda, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_4x12_vs_libc4cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG5, %r11  // B
	movq	ARG6, %r12  // sdb
	sall	$ 5, %r12d
	movq	ARG3, %r13  // A
	movq	ARG4, %r14  // lda
	sall	$ 3, %r14d

	movq	ARG12, %r15  // m1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG12, %r15  // m1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG12, %r15  // m1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4c)
#endif

103:


	// prefetch D
	// TODO prefetch vs !!!

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_4X12_LIB
#else
	CALL(inner_prefetch0_4x12_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG12, %r14   // m1
	movq	ARG13, %r15   // n1

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif


#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X12_VS_LIB
#else
	CALL(inner_scale_ab_4x12_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12   // m1
	movq	ARG13, %r13   // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_VS_LIB
#else
	CALL(inner_store_4x12_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_4x12_vs_libc4cc)





//                                  1      2              3          4        5          6        7             8          9        10         11
// void kernel_dgemm_nt_4x12_libcccc(int k, double *alpha, double *A, int lda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_4x12_libcccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG5, %r11  // B
	movq	ARG6, %r12  // ldb
	sall	$ 3, %r12d
	movq	ARG3, %r13  // A
	movq	ARG4, %r14 // lda
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIBCC
#else
	CALL(inner_kernel_dgemm_nt_12x4_libcc)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif


	// prefetch D

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_4X12_LIB
#else
	CALL(inner_prefetch0_4x12_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X12_LIB
#else
	CALL(inner_scale_ab_4x12_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_LIB
#else
	CALL(inner_store_4x12_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_4x12_libcccc)





//                                    1      2              3          4        5          6        7             8          9        10         11       12      13
// void kernel_dgemm_nt_4x4_vs_libcccc(int k, double *alpha, double *A, int lda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_4x12_vs_libcccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG5, %r11  // B
	movq	ARG6, %r12  // ldb*sizeof(double)
	sall	$ 3, %r12d
	movq	ARG3, %r13  // A
	movq	ARG4, %r14 // sda
	sall	$ 3, %r14d // sda*sizeof(double)
	movq	ARG13, %r15  // n1

	movq	ARG12, %rax  // m1
	cmpl	$ 1, %eax
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X1_VS_LIBCC
#else
	CALL(inner_kernel_dgemm_nt_12x1_vs_libcc)
#endif
	
	jmp		103f

100:

	movq	ARG12, %rax  // m1
	cmpl	$ 2, %eax
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X2_VS_LIBCC
#else
	CALL(inner_kernel_dgemm_nt_12x2_vs_libcc)
#endif

	jmp		103f

101:

	movq	ARG12, %rax  // m1
	cmpl	$ 3, %eax
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X3_VS_LIBCC
#else
	CALL(inner_kernel_dgemm_nt_12x3_vs_libcc)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_VS_LIBCC
#else
	CALL(inner_kernel_dgemm_nt_12x4_vs_libcc)
#endif

103:

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif


	// prefetch D
	// TODO prefetch vs !!!

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_4X12_LIB
#else
	CALL(inner_prefetch0_4x12_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG12, %r14 // m1
	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X12_VS_LIB
#else
	CALL(inner_scale_ab_4x12_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_VS_LIB
#else
	CALL(inner_store_4x12_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_4x12_vs_libcccc)





//                                 1      2              3          4          5        6             7          8        9          10
// void kernel_dgemm_nn_4x12_lib4ccc(int k, double *alpha, double *A, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nn_4x12_lib4ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B
	movq	ARG5, %r13  // ldb
	sall	$ 3, %r13d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X12_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_4x12_lib4c)
#endif


	// prefetch D

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_4X12_LIB
#else
	CALL(inner_prefetch0_4x12_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X12_LIB
#else
	CALL(inner_scale_ab_4x12_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_LIB
#else
	CALL(inner_store_4x12_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nn_4x12_lib4ccc)





//                                    1      2              3          4          5        6             7          8        9          10       11      12
// void kernel_dgemm_nn_4x12_vs_lib4ccc(int k, double *alpha, double *A, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nn_4x12_vs_lib4ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B
	movq	ARG5, %r13  // ldb
	sall	$ 3, %r13d

	movq	ARG12, %r14  // n1
	cmpl	$ 9, %r14d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X9_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_4x9_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG12, %r14  // n1
	cmpl	$ 10, %r14d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X10_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_4x10_lib4c)
#endif

	jmp		103f

101:

	movq	ARG12, %r14  // n1
	cmpl	$ 11, %r14d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X11_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_4x11_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X12_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_4x12_lib4c)
#endif

103:


	// prefetch D
	// TODO prefetch vs !!!

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_4X12_LIB
#else
	CALL(inner_prefetch0_4x12_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG11, %r14 // m1
	movq	ARG12, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X12_VS_LIB
#else
	CALL(inner_scale_ab_4x12_vs_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG11, %r12 // m1
	movq	ARG12, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_VS_LIB
#else
	CALL(inner_store_4x12_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nn_4x12_vs_lib4ccc)





//                                  1      2              3          4        5          6        7             8          9        10         11
// void kernel_dgemm_nn_4x12_libcccc(int k, double *alpha, double *A, int lda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nn_4x12_libcccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // lda
	sall	$ 3, %r12d
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X12_LIBCC
#else
	CALL(inner_kernel_dgemm_nn_4x12_libcc)
#endif


	// prefetch D

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_4X12_LIB
#else
	CALL(inner_prefetch0_4x12_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X12_LIB
#else
	CALL(inner_scale_ab_4x12_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_LIB
#else
	CALL(inner_store_4x12_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nn_4x12_libcccc)





//                                    1      2              3          4        5          6        7             8          9        10         11       12      13
// void kernel_dgemm_nt_4x12_vs_libcccc(int k, double *alpha, double *A, int lda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nn_4x12_vs_libcccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 3, %r12d // sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb*sizeof(double)
	sall	$ 3, %r14d
	movq	ARG12, %r15  // m1

	movq	ARG13, %rax  // n1
	cmpl	$ 9, %eax
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X9_VS_LIBCC
#else
	CALL(inner_kernel_dgemm_nn_4x9_vs_libcc)
#endif
	
	jmp		103f

100:

	movq	ARG13, %rax  // n1
	cmpl	$ 10, %eax
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X10_VS_LIBCC
#else
	CALL(inner_kernel_dgemm_nn_4x10_vs_libcc)
#endif

	jmp		103f

101:

	movq	ARG13, %rax  // n1
	cmpl	$ 11, %eax
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X11_VS_LIBCC
#else
	CALL(inner_kernel_dgemm_nn_4x11_vs_libcc)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X12_VS_LIBCC
#else
	CALL(inner_kernel_dgemm_nn_4x12_vs_libcc)
#endif

103:


	// prefetch D
	// TODO prefetch vs !!!

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_4X12_LIB
#else
	CALL(inner_prefetch0_4x12_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG12, %r14 // m1
	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X12_VS_LIB
#else
	CALL(inner_scale_ab_4x12_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_VS_LIB
#else
	CALL(inner_store_4x12_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nn_4x12_vs_libcccc)





//                                  1      2              3          4        5          6        7             8          9        10         11
// void kernel_dgemm_tt_4x12_libc4cc(int k, double *alpha, double *A, int lda, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_tt_4x12_libc4cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// prefetch C

	movq		ARG7, %r10 // beta
	vmovsd		0(%r10), %xmm14
	vxorpd		%xmm15, %xmm15, %xmm15 // 0.0
	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			100f // end

	movq	ARG8, %r10 // C
	movq	ARG9, %r11 // ldc
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_4X12_LIB
#else
	CALL(inner_prefetch0_4x12_lib)
#endif

100:


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG5, %r11  // B
	movq	ARG6, %r12  // sdb
	sall	$ 5, %r12d
	movq	ARG3, %r13  // A
	movq	ARG4, %r14  // lda
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif


	// prefetch D

//	movq	ARG10, %r10 // D
//	movq	ARG11, %r11 // ldd
//	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
//	INNER_PREFETCH0_4X12_LIB
#else
//	CALL(inner_prefetch0_4x12_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif


#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X12_LIB
#else
	CALL(inner_scale_ab_4x12_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_LIB
#else
	CALL(inner_store_4x12_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_tt_4x12_libc4cc)





//                                     1      2              3          4        5          6        7             8          9        10         11       12      13
// void kernel_dgemm_tt_4x12_vs_libc4cc(int k, double *alpha, double *A, int lda, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_tt_4x12_vs_libc4cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG5, %r11  // B
	movq	ARG6, %r12  // sdb
	sall	$ 5, %r12d
	movq	ARG3, %r13  // A
	movq	ARG4, %r14  // lda
	sall	$ 3, %r14d

	movq	ARG12, %r15  // m1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG12, %r15  // m1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG12, %r15  // m1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif

103:


	// prefetch D
	// TODO prefetch vs !!!

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_4X12_LIB
#else
	CALL(inner_prefetch0_4x12_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG12, %r14   // m1
	movq	ARG13, %r15   // n1

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif


#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X12_VS_LIB
#else
	CALL(inner_scale_ab_4x12_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12   // m1
	movq	ARG13, %r13   // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_VS_LIB
#else
	CALL(inner_store_4x12_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_tt_4x12_vs_libc4cc)





//                                     1      2          3        4          5        6          7        8          9        10
// void kernel_dgetrf_nn_l_12x4_lib4ccc(int k, double *A, int sda, double *B, int ldb, double *C, int ldc, double *D, int ldd, double *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetrf_nn_l_12x4_lib4ccc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13  // B
	movq	ARG5, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // C
	movq	ARG7, %r11 // ldc
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB
#else
	CALL(inner_scale_m11_12x4_lib)
#endif


	// factorization

	movq	ARG10, %r10  // inv_diag_D 

#if MACRO_LEVEL>=1
	INNER_EDGE_DGETRF_L_12X4_LIB4
#else
	CALL(inner_edge_dgetrf_l_12x4_lib4)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgetrf_nn_l_12x4_lib4ccc)





//                                        1      2          3        4          5        6          7        8          9        10                  11      12
// void kernel_dgetrf_nn_l_12x4_vs_lib4ccc(int k, double *A, int sda, double *B, int ldb, double *C, int ldc, double *D, int ldd, double *inv_diag_D, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetrf_nn_l_12x4_vs_lib4ccc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13  // B
	movq	ARG5, %r14  // ldb
	sall	$ 3, %r14d

	movq	ARG12, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG12, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG12, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif

103:


	// call inner blender_loader nn

	movq	ARG6, %r10 // C
	movq	ARG7, %r11 // ldc
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB
#else
	CALL(inner_scale_m11_12x4_lib)
#endif


	// factorization

	movq	ARG10, %r10  // inv_diag_D 

#if MACRO_LEVEL>=1
	INNER_EDGE_DGETRF_L_12X4_LIB4
#else
	CALL(inner_edge_dgetrf_l_12x4_lib4)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG11, %r12 // m1
	movq	ARG12, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgetrf_nn_l_12x4_vs_lib4ccc)





//                                     1      2          3        4          5        6          7        8          9        10
// void kernel_dgetrf_nn_m_12x4_lib4ccc(int k, double *A, int sda, double *B, int ldb, double *C, int ldc, double *D, int ldd, double *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetrf_nn_m_12x4_lib4ccc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13  // B
	movq	ARG5, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // C
	movq	ARG7, %r11 // ldc
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB
#else
	CALL(inner_scale_m11_12x4_lib)
#endif


	// factorization

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // ldd
	sall	$ 3, %r11d // ldd*sizeof(double)
	movq	ARG10, %r12  // inv_diag_D 

#if MACRO_LEVEL>=1
	INNER_EDGE_DGETRF_M_12X4_LIB
#else
	CALL(inner_edge_dgetrf_m_12x4_lib)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgetrf_nn_m_12x4_lib4ccc)





//                                        1      2          3        4          5        6          7        8          9        10                  11      12
// void kernel_dgetrf_nn_m_12x4_vs_lib4ccc(int k, double *A, int sda, double *B, int ldb, double *C, int ldc, double *D, int ldd, double *inv_diag_D, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetrf_nn_m_12x4_vs_lib4ccc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13  // B
	movq	ARG5, %r14  // ldb
	sall	$ 3, %r14d

	movq	ARG12, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG12, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG12, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif

103:


	// call inner blender_loader nn

	movq	ARG6, %r10 // C
	movq	ARG7, %r11 // ldc
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB
#else
	CALL(inner_scale_m11_12x4_lib)
#endif


	// factorization

	movq	ARG8, %r10  // D
	movq	ARG9, %r11 // ldd
	sall	$ 3, %r11d // ldd*sizeof(double)
	movq	ARG10, %r12  // inv_diag_D 

#if MACRO_LEVEL>=1
	INNER_EDGE_DGETRF_M_12X4_LIB
#else
	CALL(inner_edge_dgetrf_m_12x4_lib)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG11, %r12 // m1
	movq	ARG12, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgetrf_nn_m_12x4_vs_lib4ccc)





//                                     1      2          3        4          5        6          7        8          9        10
// void kernel_dgetrf_nn_r_12x4_lib4ccc(int k, double *A, int sda, double *B, int ldb, double *C, int ldc, double *D, int ldd, double *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetrf_nn_r_12x4_lib4ccc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13  // B
	movq	ARG5, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // C
	movq	ARG7, %r11 // ldc
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB
#else
	CALL(inner_scale_m11_12x4_lib)
#endif


	// factorization

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // ldd
	sall	$ 3, %r11d // ldd*sizeof(double)
	movq	ARG10, %r12  // inv_diag_D 

#if MACRO_LEVEL>=1
	INNER_EDGE_DGETRF_R_12X4_LIB
#else
	CALL(inner_edge_dgetrf_r_12x4_lib)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgetrf_nn_r_12x4_lib4ccc)





//                                        1      2          3        4          5        6          7        8          9        10                  11      12
// void kernel_dgetrf_nn_r_12x4_vs_lib4ccc(int k, double *A, int sda, double *B, int ldb, double *C, int ldc, double *D, int ldd, double *inv_diag_D, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetrf_nn_r_12x4_vs_lib4ccc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13  // B
	movq	ARG5, %r14  // ldb
	sall	$ 3, %r14d

	movq	ARG12, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X1_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x1_lib4c)
#endif
	
	jmp		103f

100:

	movq	ARG12, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X2_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x2_lib4c)
#endif

	jmp		103f

101:

	movq	ARG12, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X3_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x3_lib4c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4C
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4c)
#endif

103:


	// call inner blender_loader nn

	movq	ARG6, %r10 // C
	movq	ARG7, %r11 // ldc
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB
#else
	CALL(inner_scale_m11_12x4_lib)
#endif


	// factorization

	movq	ARG8, %r10  // D
	movq	ARG9, %r11 // ldd
	sall	$ 3, %r11d // ldd*sizeof(double)
	movq	ARG10, %r12  // inv_diag_D 

#if MACRO_LEVEL>=1
	INNER_EDGE_DGETRF_R_12X4_LIB
#else
	CALL(inner_edge_dgetrf_r_12x4_lib)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG11, %r12 // m1
	movq	ARG12, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgetrf_nn_r_12x4_vs_lib4ccc)





//                                      1      2      3              4          5        6          7        8             9          10       11         12       13           14
// void kernel_dgemm_nt_12xn_p0_lib44cc(int n, int k, double *alpha, double *A, int sda, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd, double *A_p, double *B_p);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_12xn_p0_lib44cc)

	PROLOGUE

	// zero accumulation registers


//	movq	ARG6, %rbp // B
//	movq	ARG9, %rax // C
//	movq	ARG11, %rbx // D


	// loop over n
//1001:
	cmpq	$ 3, ARG1
	jle		1000f // consider clean-up loop
1001:


	ZERO_ACC


	// prefetch C

	prefetcht1	0(ARG3) // alpha
	movq		ARG8, %r10 // beta
	vmovsd		0(%r10), %xmm14
	vxorpd		%xmm15, %xmm15, %xmm15 // 0.0
	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			100f // end

	movq	ARG9, %r10 // C
	movq	ARG10, %r11 // ldc
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH1_12X4_LIB
#else
	CALL(inner_prefetch1_12x4_lib)
#endif

100:


	// call inner dgemm kernel nn

	movq	ARG2, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG6, %r13  // B

	movq	ARG4, %r14  // A_p
	cmpq	$ 4, ARG1
	cmoveq	ARG13, %r14  // A_p

	movq	ARG7, %r15 // sdb
	sall	$ 5, %r15d // 4*sdb*sizeof(double)
	addq	ARG6, %r15 // B_p
	cmpq	$ 4, ARG1
	cmoveq	ARG14, %r15  // B_p

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_P0_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_p0_lib4)
#endif



	movq	ARG11, %r10 // D
	cmpq	ARG9, %r10
	jne			100f // end

	movq	ARG12, %r10 // ldd
	cmpq	ARG10, %r10
	jne			100f // end

	movq	ARG8, %r10 // beta
	vbroadcastsd	0(%r10), %ymm14
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC05(%rip), %ymm15 // 1.0
#else
	vmovapd		LC05(%rip), %ymm15 // 1.0
#endif
	vucomisd	%xmm15, %xmm14 // beta==1.0 ?
	jne			100f // end



	movq	ARG3, %r10 // alpha
	movq	ARG11, %r11   // D
	movq	ARG12, %r12   // ldd
	sall	$ 3, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_A1_STORE_12X4_LIB
#else
	CALL(inner_scale_a1_store_12x4_lib)
#endif


	jmp		101f


100:


	// call inner blend

	movq	ARG3, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12   // C
	movq	ARG10, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


101:


	// loop increments
	// n
//	movzx	%edi, %rdi //ARG1
	subq	$ 4, ARG1
	// B
	movq	ARG7, %r10
	sall	$ 5, %r10d
	addq	%r10, ARG6
	// C
	movq	ARG10, %r10
	sall	$ 5, %r10d
	addq	%r10, ARG9
	// D
	movq	ARG12, %r10
	sall	$ 5, %r10d
	addq	%r10, ARG11


	cmpq	$ 3, ARG1
	jg		1001b // main loop


1000:
	// TODO clean-up
	cmpq	$ 0, ARG1
	jle		1000f // consider clean-up loop



	ZERO_ACC


	// prefetch C

	prefetcht1	0(ARG3) // alpha
	movq		ARG8, %r10 // beta
	vmovsd		0(%r10), %xmm14
	vxorpd		%xmm15, %xmm15, %xmm15 // 0.0
	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			100f // end

	movq	ARG9, %r10 // C
	movq	ARG10, %r11 // ldc
	sall	$ 3, %r11d

	// TODO prefetch vs

#if MACRO_LEVEL>=1
	INNER_PREFETCH1_12X4_LIB
#else
	CALL(inner_prefetch1_12x4_lib)
#endif

100:


	// call inner dgemm kernel nn

	movq	ARG2, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG6, %r13  // B
	movq	ARG4, %r14  // A_p
	addq	ARG6, %r15 // B_p

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_P0_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_p0_lib4)
#endif




	// call inner blend

	movq	ARG3, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12   // C
	movq	ARG10, %r13   // ldc
	sall	$ 3, %r13d
	movq	$ 12, %r14   // m1
	movq	ARG1, %r15   // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // ldd
	sall	$ 3, %r11d
	movq	$ 12, %r12   // m1
	movq	ARG1, %r13   // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif







	// loop increments
#if 0
	// n
//	movzx	%edi, %rdi //ARG1
	subq	$ 4, ARG1
	// B
	movq	ARG7, %r10
	sall	$ 5, %r10d
	addq	%r10, ARG6
	// C
	movq	ARG10, %r10
	sall	$ 5, %r10d
	addq	%r10, ARG9
	// D
	movq	ARG12, %r10
	sall	$ 5, %r10d
	addq	%r10, ARG11
#endif



1000:
	// return

	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_12xn_p0_lib44cc)





//                                      1      2      3              4          5        6          7        8             9          10       11         12       13           14
// void kernel_dgemm_nt_12xn_pl_lib44cc(int n, int k, double *alpha, double *A, int sda, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd, double *A_p, double *B_p);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_12xn_pl_lib44cc)

	PROLOGUE

	// zero accumulation registers


//	movq	ARG6, %rbp // B
//	movq	ARG9, %rax // C
//	movq	ARG11, %rbx // D


	// loop over n
//1001:
//	cmpq	$ 3, ARG1
	cmpq	$ 4, ARG1
	jle		1000f // consider clean-up loop

1001:


	ZERO_ACC


	// prefetch C

	prefetcht1	0(ARG3) // alpha
	movq		ARG8, %r10 // beta
	vmovsd		0(%r10), %xmm14
	vxorpd		%xmm15, %xmm15, %xmm15 // 0.0
	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			100f // end

	movq	ARG9, %r10 // C
	movq	ARG10, %r11 // ldc
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH1_12X4_LIB
#else
	CALL(inner_prefetch1_12x4_lib)
#endif

100:


	// call inner dgemm kernel nn

	movq	ARG2, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG6, %r13  // B

	movq	ARG4, %r14  // A_p
	cmpq	$ 4, ARG1
	cmoveq	ARG13, %r14  // A_p

	movq	ARG7, %r15 // sdb
	sall	$ 5, %r15d // 4*sdb*sizeof(double)
	addq	ARG6, %r15 // B_p
	cmpq	$ 4, ARG1
	cmoveq	ARG14, %r15  // B_p

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_P0_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_p0_lib4)
#endif



	movq	ARG11, %r10 // D
	cmpq	ARG9, %r10
	jne			100f // end

	movq	ARG12, %r10 // ldd
	cmpq	ARG10, %r10
	jne			100f // end

	movq	ARG8, %r10 // beta
	vbroadcastsd	0(%r10), %ymm14
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC05(%rip), %ymm15 // 1.0
#else
	vmovapd		LC05(%rip), %ymm15 // 1.0
#endif
	vucomisd	%xmm15, %xmm14 // beta==1.0 ?
	jne			100f // end



	movq	ARG3, %r10 // alpha
	movq	ARG11, %r11   // D
	movq	ARG12, %r12   // ldd
	sall	$ 3, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_A1_STORE_12X4_LIB
#else
	CALL(inner_scale_a1_store_12x4_lib)
#endif


	jmp		101f


100:


	// call inner blend

	movq	ARG3, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12   // C
	movq	ARG10, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


101:


	// loop increments
	// n
//	movzx	%edi, %rdi //ARG1
	subq	$ 4, ARG1
	// B
	movq	ARG7, %r10
	sall	$ 5, %r10d
	addq	%r10, ARG6
	// C
	movq	ARG10, %r10
	sall	$ 5, %r10d
	addq	%r10, ARG9
	// D
	movq	ARG12, %r10
	sall	$ 5, %r10d
	addq	%r10, ARG11


//	cmpq	$ 3, ARG1
	cmpq	$ 4, ARG1
	jg		1001b // main loop


1000:
	// TODO clean-up
	cmpq	$ 3, ARG1
//	cmpq	$ 0, ARG1
	jle		1000f // consider clean-up loop


	ZERO_ACC


	// prefetch C

	prefetcht1	0(ARG3) // alpha
	movq		ARG8, %r10 // beta
	vmovsd		0(%r10), %xmm14
	vxorpd		%xmm15, %xmm15, %xmm15 // 0.0
	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			100f // end

	movq	ARG9, %r10 // C
	movq	ARG10, %r11 // ldc
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH1_12X4_LIB
#else
	CALL(inner_prefetch1_12x4_lib)
#endif

100:


	// call inner dgemm kernel nn

	movq	ARG2, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG6, %r13  // B

//	movq	ARG4, %r14  // A_p
//	cmpq	$ 4, ARG1
//	cmoveq	ARG13, %r14  // A_p
	mov		ARG13, %r14  // A_p

//	movq	ARG7, %r15 // sdb
//	sall	$ 5, %r15d // 4*sdb*sizeof(double)
//	addq	ARG6, %r15 // B_p
//	cmpq	$ 4, ARG1
//	cmoveq	ARG14, %r15  // B_p
	mov		ARG14, %r15  // B_p

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_PL_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_pl_lib4)
#endif



	movq	ARG11, %r10 // D
	cmpq	ARG9, %r10
	jne			100f // end

	movq	ARG12, %r10 // ldd
	cmpq	ARG10, %r10
	jne			100f // end

	movq	ARG8, %r10 // beta
	vbroadcastsd	0(%r10), %ymm14
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC05(%rip), %ymm15 // 1.0
#else
	vmovapd		LC05(%rip), %ymm15 // 1.0
#endif
	vucomisd	%xmm15, %xmm14 // beta==1.0 ?
	jne			100f // end



	movq	ARG3, %r10 // alpha
	movq	ARG11, %r11   // D
	movq	ARG12, %r12   // ldd
	sall	$ 3, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_A1_STORE_12X4_LIB
#else
	CALL(inner_scale_a1_store_12x4_lib)
#endif


	jmp		101f


100:


	// call inner blend

	movq	ARG3, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12   // C
	movq	ARG10, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB
#else
	CALL(inner_store_12x4_lib)
#endif


101:


#if 0
	// loop increments
	// n
//	movzx	%edi, %rdi //ARG1
	subq	$ 4, ARG1
	// B
	movq	ARG7, %r10
	sall	$ 5, %r10d
	addq	%r10, ARG6
	// C
	movq	ARG10, %r10
	sall	$ 5, %r10d
	addq	%r10, ARG9
	// D
	movq	ARG12, %r10
	sall	$ 5, %r10d
	addq	%r10, ARG11
#endif


	jmp		2000f // return



1000:
	// TODO clean-up
	cmpq	$ 0, ARG1
	jle		2000f // consider clean-up loop



	ZERO_ACC


	// prefetch C

	prefetcht1	0(ARG3) // alpha
	movq		ARG8, %r10 // beta
	vmovsd		0(%r10), %xmm14
	vxorpd		%xmm15, %xmm15, %xmm15 // 0.0
	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			100f // end

	movq	ARG9, %r10 // C
	movq	ARG10, %r11 // ldc
	sall	$ 3, %r11d

	// TODO prefetch vs

#if MACRO_LEVEL>=1
	INNER_PREFETCH1_12X4_LIB
#else
	CALL(inner_prefetch1_12x4_lib)
#endif

100:


	// call inner dgemm kernel nn

	movq	ARG2, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG6, %r13  // B
	movq	ARG4, %r14  // A_p
	addq	ARG6, %r15 // B_p

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_P0_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_p0_lib4)
#endif




	// call inner blend

	movq	ARG3, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12   // C
	movq	ARG10, %r13   // ldc
	sall	$ 3, %r13d
	movq	$ 12, %r14   // m1
	movq	ARG1, %r15   // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // ldd
	sall	$ 3, %r11d
	movq	$ 12, %r12   // m1
	movq	ARG1, %r13   // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif







	// loop increments
#if 0
	// n
//	movzx	%edi, %rdi //ARG1
	subq	$ 4, ARG1
	// B
	movq	ARG7, %r10
	sall	$ 5, %r10d
	addq	%r10, ARG6
	// C
	movq	ARG10, %r10
	sall	$ 5, %r10d
	addq	%r10, ARG9
	// D
	movq	ARG12, %r10
	sall	$ 5, %r10d
	addq	%r10, ARG11
#endif



2000:
	// return

	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_12xn_pl_lib44cc)





//                                      1      2      3              4          5          6        7        8             9          10       11         12       13           14
// void kernel_dgemm_nt_mx12_p0_lib44cc(int m, int k, double *alpha, double *A, int sda, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd, double *A_p, double *B_p);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_mx12_p0_lib44cc)

	PROLOGUE

	// zero accumulation registers


//	movq	ARG6, %rbp // B
//	movq	ARG9, %rax // C
//	movq	ARG11, %rbx // D


	// loop over n
//1001:
	cmpq	$ 3, ARG1
	jle		1000f // consider clean-up loop
1001:


	ZERO_ACC


	// prefetch C

	prefetcht1	0(ARG3) // alpha
	movq		ARG8, %r10 // beta
	vmovsd		0(%r10), %xmm14
	vxorpd		%xmm15, %xmm15, %xmm15 // 0.0
	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			100f // end

	movq	ARG9, %r10 // C
	movq	ARG10, %r11 // ldc
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH1_4X12_LIB
#else
	CALL(inner_prefetch1_4x12_lib)
#endif

100:


	// call inner dgemm kernel nn

	movq	ARG2, %r10 // k
	movq	ARG6, %r11  // B
	movq	ARG7, %r12 // sdb
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG4, %r13  // A

	movq	ARG6, %r14  // B_p
	cmpq	$ 4, ARG1
	cmoveq	ARG14, %r14  // B_p

	movq	ARG5, %r15 // sda
	sall	$ 5, %r15d // 4*sda*sizeof(double)
	addq	ARG4, %r15 // A_p
	cmpq	$ 4, ARG1
	cmoveq	ARG13, %r15  // A_p

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_P0_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_p0_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif



	movq	ARG11, %r10 // D
	cmpq	ARG9, %r10
	jne			100f // end

	movq	ARG12, %r10 // ldd
	cmpq	ARG10, %r10
	jne			100f // end

	movq	ARG8, %r10 // beta
	vbroadcastsd	0(%r10), %ymm14
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC05(%rip), %ymm15 // 1.0
#else
	vmovapd		LC05(%rip), %ymm15 // 1.0
#endif
	vucomisd	%xmm15, %xmm14 // beta==1.0 ?
	jne			100f // end



	movq	ARG3, %r10 // alpha
	movq	ARG11, %r11   // D
	movq	ARG12, %r12   // ldd
	sall	$ 3, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_A1_STORE_4X12_LIB
#else
	CALL(inner_scale_a1_store_4x12_lib)
#endif


	jmp		101f


100:


	// call inner blend

	movq	ARG3, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12   // C
	movq	ARG10, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X12_LIB
#else
	CALL(inner_scale_ab_4x12_lib)
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_LIB
#else
	CALL(inner_store_4x12_lib)
#endif


101:


	// loop increments
	// n
//	movzx	%edi, %rdi //ARG1
	subq	$ 4, ARG1
	// A
	movq	ARG5, %r10
	sall	$ 5, %r10d
	addq	%r10, ARG4
	// C
//	movq	ARG10, %r10
//	sall	$ 5, %r10d
//	addq	%r10, ARG9
	addq	$ 32, ARG9
	// D
//	movq	ARG12, %r10
//	sall	$ 5, %r10d
//	addq	%r10, ARG11
	addq	$ 32, ARG11


	cmpq	$ 3, ARG1
	jg		1001b // main loop


1000:
	// TODO clean-up
	cmpq	$ 0, ARG1
	jle		1000f // consider clean-up loop



	ZERO_ACC


	// prefetch C

	prefetcht1	0(ARG3) // alpha
	movq		ARG8, %r10 // beta
	vmovsd		0(%r10), %xmm14
	vxorpd		%xmm15, %xmm15, %xmm15 // 0.0
	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			100f // end

	movq	ARG9, %r10 // C
	movq	ARG10, %r11 // ldc
	sall	$ 3, %r11d

	// TODO prefetch vs

#if MACRO_LEVEL>=1
	INNER_PREFETCH1_4X12_LIB
#else
	CALL(inner_prefetch1_4x12_lib)
#endif

100:


	// call inner dgemm kernel nn

	movq	ARG2, %r10 // k
	movq	ARG6, %r11  // B
	movq	ARG7, %r12 // sdb
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG4, %r13  // A
	movq	ARG6, %r14  // B_p
	addq	ARG4, %r15 // A_p

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_P0_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_p0_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB
#else
	CALL(inner_tran_12x4_lib)
#endif



	// call inner blend

	movq	ARG3, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12   // C
	movq	ARG10, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG1, %r14   // m1
	movq	$ 12, %r15   // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X12_VS_LIB
#else
	CALL(inner_scale_ab_4x12_vs_lib)
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG1, %r12   // m1
	movq	$ 12, %r13   // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_VS_LIB
#else
	CALL(inner_store_4x12_vs_lib)
#endif







	// loop increments
#if 0
	// n
//	movzx	%edi, %rdi //ARG1
	subq	$ 4, ARG1
	// A
	movq	ARG5, %r10
	sall	$ 5, %r10d
	addq	%r10, ARG4
	// C
//	movq	ARG10, %r10
//	sall	$ 5, %r10d
//	addq	%r10, ARG9
	addq	$ 32, ARG9
	// D
//	movq	ARG12, %r10
//	sall	$ 5, %r10d
//	addq	%r10, ARG11
	addq	$ 32, ARG11
#endif



1000:
	// return

	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_mx12_p0_lib44cc)






