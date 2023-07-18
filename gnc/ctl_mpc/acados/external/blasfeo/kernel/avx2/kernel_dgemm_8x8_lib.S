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
// scale for generic alpha= and beta=
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- ldc
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
	.macro INNER_SCALE_AB_8X8_LIB
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_8x8_lib)
#endif

	vbroadcastsd 0(%r10), %ymm15 // alpha

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

	vbroadcastsd 0(%r11), %ymm14 // beta

	vxorpd		%ymm15, %ymm15, %ymm15 // 0.0

	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm0
	vmovupd		32(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm4
	addq		%r13, %r12
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm1
	vmovupd		32(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm5
	addq		%r13, %r12
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm2
	vmovupd		32(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm6
	addq		%r13, %r12
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm3
	vmovupd		32(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm7
	addq		%r13, %r12

	vmovupd		32(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm8
	addq		%r13, %r12
	vmovupd		32(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm9
	addq		%r13, %r12
	vmovupd		32(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm10
	addq		%r13, %r12
	vmovupd		32(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm11
//	addq		%r13, %r12

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_8x8_lib)
#endif





// common inner routine with file scope
//
// transpose scale for generic alpha= and beta=
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- ldc
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
	.macro INNER_TRAN_SCALE_AB_8X8_LIB
#else
	.p2align 4,,15
	FUN_START(inner_tran_scale_ab_8x8_lib)
#endif

	vunpcklpd	%ymm1, %ymm0, %ymm12
	vunpckhpd	%ymm1, %ymm0, %ymm13
	vunpcklpd	%ymm3, %ymm2, %ymm14
	vunpckhpd	%ymm3, %ymm2, %ymm15

	vperm2f128	$ 0x20, %ymm14, %ymm12, %ymm0
	vperm2f128	$ 0x31, %ymm14, %ymm12, %ymm2
	vperm2f128	$ 0x20, %ymm15, %ymm13, %ymm1
	vperm2f128	$ 0x31, %ymm15, %ymm13, %ymm3

	vbroadcastsd 0(%r10), %ymm15 // alpha

	vmulpd		%ymm0, %ymm15, %ymm0
	vmulpd		%ymm1, %ymm15, %ymm1
	vmulpd		%ymm2, %ymm15, %ymm2
	vmulpd		%ymm3, %ymm15, %ymm3

	vunpcklpd	%ymm5, %ymm4, %ymm12
	vunpckhpd	%ymm5, %ymm4, %ymm13
	vunpcklpd	%ymm7, %ymm6, %ymm14
	vunpckhpd	%ymm7, %ymm6, %ymm15

	vperm2f128	$ 0x20, %ymm14, %ymm12, %ymm4
	vperm2f128	$ 0x31, %ymm14, %ymm12, %ymm6
	vperm2f128	$ 0x20, %ymm15, %ymm13, %ymm5
	vperm2f128	$ 0x31, %ymm15, %ymm13, %ymm7

	vbroadcastsd 0(%r10), %ymm15 // alpha

	vmulpd		%ymm4, %ymm15, %ymm4
	vmulpd		%ymm5, %ymm15, %ymm5
	vmulpd		%ymm6, %ymm15, %ymm6
	vmulpd		%ymm7, %ymm15, %ymm7

	vunpcklpd	%ymm9, %ymm8, %ymm12
	vunpckhpd	%ymm9, %ymm8, %ymm13
	vunpcklpd	%ymm11, %ymm10, %ymm14
	vunpckhpd	%ymm11, %ymm10, %ymm15

	vperm2f128	$ 0x20, %ymm14, %ymm12, %ymm8
	vperm2f128	$ 0x31, %ymm14, %ymm12, %ymm10
	vperm2f128	$ 0x20, %ymm15, %ymm13, %ymm9
	vperm2f128	$ 0x31, %ymm15, %ymm13, %ymm11

	vbroadcastsd 0(%r10), %ymm15 // alpha

	vmulpd		%ymm8, %ymm15, %ymm8
	vmulpd		%ymm9, %ymm15, %ymm9
	vmulpd		%ymm10, %ymm15, %ymm10
	vmulpd		%ymm11, %ymm15, %ymm11

	vbroadcastsd 0(%r11), %ymm14 // beta

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
	vmovupd		32(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm8
	addq		%r13, %r12
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm5
	vmovupd		32(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm9
	addq		%r13, %r12
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm6
	vmovupd		32(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm10
	addq		%r13, %r12
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm7
	vmovupd		32(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm11
//	addq		%r13, %r12

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_tran_scale_ab_8x8_lib)
#endif





// common inner routine with file scope
//
// scale for generic alpha= and beta=
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- ldc
// r14d   <- km
// r15d   <- kn
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
	.macro INNER_SCALE_AB_8X8_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_8x8_vs_lib)
#endif

	vbroadcastsd 0(%r10), %ymm15 // alpha

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

	vbroadcastsd 0(%r11), %ymm14 // beta

	vxorpd		%ymm15, %ymm15, %ymm15 // 0.0

	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end


	vcvtsi2sd	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC03(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC03(%rip), %ymm13
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm13, %ymm13


	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm0
	vmaskmovpd	32(%r12), %ymm13, %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm4
	addq		%r13, %r12
//	cmpl		$ 2, %r15d
//	jl			0f // end
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm1
	vmaskmovpd	32(%r12), %ymm13, %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm5
	addq		%r13, %r12
//	cmpl		$ 3, %r15d
//	jl			0f // end
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm2
	vmaskmovpd	32(%r12), %ymm13, %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm6
	addq		%r13, %r12
//	cmpl		$ 4, %r15d
//	jl			0f // end
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm3
	vmaskmovpd	32(%r12), %ymm13, %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm7
	addq		%r13, %r12
//	cmpl		$ 5, %r15d
//	jl			0f // end

	vmaskmovpd	32(%r12), %ymm13, %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm8
	addq		%r13, %r12
	cmpl		$ 6, %r15d
	jl			0f // end
	vmaskmovpd	32(%r12), %ymm13, %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm9
	addq		%r13, %r12
	cmpl		$ 7, %r15d
	jl			0f // end
	vmaskmovpd	32(%r12), %ymm13, %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm10
	addq		%r13, %r12
	cmpl		$ 7, %r15d
	je			0f // end
	vmaskmovpd	32(%r12), %ymm13, %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm11
//	addq		%r13, %r12

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_8x8_vs_lib)
#endif





// common inner routine with file scope
//
// scale for generic alpha= and beta=
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- ldc
// r14d   <- km
// r15d   <- kn
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
	.macro INNER_TRAN_SCALE_AB_8X8_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_tran_scale_ab_8x8_vs_lib)
#endif

	vunpcklpd	%ymm1, %ymm0, %ymm12
	vunpckhpd	%ymm1, %ymm0, %ymm13
	vunpcklpd	%ymm3, %ymm2, %ymm14
	vunpckhpd	%ymm3, %ymm2, %ymm15

	vperm2f128	$ 0x20, %ymm14, %ymm12, %ymm0
	vperm2f128	$ 0x31, %ymm14, %ymm12, %ymm2
	vperm2f128	$ 0x20, %ymm15, %ymm13, %ymm1
	vperm2f128	$ 0x31, %ymm15, %ymm13, %ymm3

	vbroadcastsd 0(%r10), %ymm15 // alpha

	vmulpd		%ymm0, %ymm15, %ymm0
	vmulpd		%ymm1, %ymm15, %ymm1
	vmulpd		%ymm2, %ymm15, %ymm2
	vmulpd		%ymm3, %ymm15, %ymm3

	vunpcklpd	%ymm5, %ymm4, %ymm12
	vunpckhpd	%ymm5, %ymm4, %ymm13
	vunpcklpd	%ymm7, %ymm6, %ymm14
	vunpckhpd	%ymm7, %ymm6, %ymm15

	vperm2f128	$ 0x20, %ymm14, %ymm12, %ymm4
	vperm2f128	$ 0x31, %ymm14, %ymm12, %ymm6
	vperm2f128	$ 0x20, %ymm15, %ymm13, %ymm5
	vperm2f128	$ 0x31, %ymm15, %ymm13, %ymm7

	vbroadcastsd 0(%r10), %ymm15 // alpha

	vmulpd		%ymm4, %ymm15, %ymm4
	vmulpd		%ymm5, %ymm15, %ymm5
	vmulpd		%ymm6, %ymm15, %ymm6
	vmulpd		%ymm7, %ymm15, %ymm7

	vunpcklpd	%ymm9, %ymm8, %ymm12
	vunpckhpd	%ymm9, %ymm8, %ymm13
	vunpcklpd	%ymm11, %ymm10, %ymm14
	vunpckhpd	%ymm11, %ymm10, %ymm15

	vperm2f128	$ 0x20, %ymm14, %ymm12, %ymm8
	vperm2f128	$ 0x31, %ymm14, %ymm12, %ymm10
	vperm2f128	$ 0x20, %ymm15, %ymm13, %ymm9
	vperm2f128	$ 0x31, %ymm15, %ymm13, %ymm11

	vbroadcastsd 0(%r10), %ymm15 // alpha

	vmulpd		%ymm8, %ymm15, %ymm8
	vmulpd		%ymm9, %ymm15, %ymm9
	vmulpd		%ymm10, %ymm15, %ymm10
	vmulpd		%ymm11, %ymm15, %ymm11

	vbroadcastsd 0(%r11), %ymm14 // beta

	vxorpd		%ymm15, %ymm15, %ymm15 // 0.0

	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end


	vcvtsi2sd	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC03(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC03(%rip), %ymm13
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm13, %ymm13


	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm0
	addq		%r13, %r12
//	cmpl		$ 2, %r15d
//	jl			0f // end
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm1
	addq		%r13, %r12
//	cmpl		$ 3, %r15d
//	jl			0f // end
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm2
	addq		%r13, %r12
//	cmpl		$ 4, %r15d
//	jl			0f // end
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm3
	addq		%r13, %r12
//	cmpl		$ 5, %r15d
//	jl			0f // end

	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm4
	vmaskmovpd	32(%r12), %ymm13, %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm8
	addq		%r13, %r12
	cmpl		$ 6, %r15d
	jl			0f // end
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm5
	vmaskmovpd	32(%r12), %ymm13, %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm9
	addq		%r13, %r12
	cmpl		$ 7, %r15d
	jl			0f // end
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm6
	vmaskmovpd	32(%r12), %ymm13, %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm10
	addq		%r13, %r12
	cmpl		$ 7, %r15d
	je			0f // end
	vmovupd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm7
	vmaskmovpd	32(%r12), %ymm13, %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm11
//	addq		%r13, %r12

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_tran_scale_ab_8x8_vs_lib)
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
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M11_8X8_LIB
#else
	.p2align 4,,15
	FUN_START(inner_scale_m11_8x8_lib)
#endif

	vmovupd		0(%r10), %ymm15
	vsubpd		%ymm0, %ymm15, %ymm0
	vmovupd		32(%r10), %ymm15
	vsubpd		%ymm4, %ymm15, %ymm4
	addq		%r11, %r10
	vmovupd		0(%r10), %ymm15
	vsubpd		%ymm1, %ymm15, %ymm1
	vmovupd		32(%r10), %ymm15
	vsubpd		%ymm5, %ymm15, %ymm5
	addq		%r11, %r10
	vmovupd		0(%r10), %ymm15
	vsubpd		%ymm2, %ymm15, %ymm2
	vmovupd		32(%r10), %ymm15
	vsubpd		%ymm6, %ymm15, %ymm6
	addq		%r11, %r10
	vmovupd		0(%r10), %ymm15
	vsubpd		%ymm3, %ymm15, %ymm3
	vmovupd		32(%r10), %ymm15
	vsubpd		%ymm7, %ymm15, %ymm7
	addq		%r11, %r10

	vmovupd		32(%r10), %ymm15
	vsubpd		%ymm8, %ymm15, %ymm8
	addq		%r11, %r10
	vmovupd		32(%r10), %ymm15
	vsubpd		%ymm9, %ymm15, %ymm9
	addq		%r11, %r10
	vmovupd		32(%r10), %ymm15
	vsubpd		%ymm10, %ymm15, %ymm10
	addq		%r11, %r10
	vmovupd		32(%r10), %ymm15
	vsubpd		%ymm11, %ymm15, %ymm11
//	addq		%r11, %r10

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m11_8x8_lib)
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
	.macro INNER_SCALE_M11_8X8_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_scale_m11_8x8_vs_lib)
#endif

	vcvtsi2sd	%r12d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC03(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC03(%rip), %ymm13
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm13, %ymm13


	vmovupd		0(%r10), %ymm15
	vsubpd		%ymm0, %ymm15, %ymm0
	vmaskmovpd	32(%r10), %ymm13, %ymm15
	vsubpd		%ymm4, %ymm15, %ymm4
	addq		%r11, %r10
	cmpl		$ 2, %r13d
	jl			0f // end
	vmovupd		0(%r10), %ymm15
	vsubpd		%ymm1, %ymm15, %ymm1
	vmaskmovpd	32(%r10), %ymm13, %ymm15
	vsubpd		%ymm5, %ymm15, %ymm5
	addq		%r11, %r10
	cmpl		$ 3, %r13d
	jl			0f // end
	vmovupd		0(%r10), %ymm15
	vsubpd		%ymm2, %ymm15, %ymm2
	vmaskmovpd	32(%r10), %ymm13, %ymm15
	vsubpd		%ymm6, %ymm15, %ymm6
	addq		%r11, %r10
	cmpl		$ 3, %r13d
	je			0f // end
	vmovupd		0(%r10), %ymm15
	vsubpd		%ymm3, %ymm15, %ymm3
	vmaskmovpd	32(%r10), %ymm13, %ymm15
	vsubpd		%ymm7, %ymm15, %ymm7
	addq		%r11, %r10

	vmaskmovpd	32(%r10), %ymm13, %ymm15
	vsubpd		%ymm8, %ymm15, %ymm8
	addq		%r11, %r10
	cmpl		$ 2, %r13d
	jl			0f // end
	vmaskmovpd	32(%r10), %ymm13, %ymm15
	vsubpd		%ymm9, %ymm15, %ymm9
	addq		%r11, %r10
	cmpl		$ 3, %r13d
	jl			0f // end
	vmaskmovpd	32(%r10), %ymm13, %ymm15
	vsubpd		%ymm10, %ymm15, %ymm10
	addq		%r11, %r10
	cmpl		$ 3, %r13d
	je			0f // end
	vmaskmovpd	32(%r10), %ymm13, %ymm15
	vsubpd		%ymm11, %ymm15, %ymm11
//	addq		%r11, %r10

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m11_8x8_vs_lib)
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
	.macro INNER_STORE_L_8X8_LIB
#else
	.p2align 4,,15
	FUN_START(inner_store_l_8x8_lib)
#endif

	vmovupd		%ymm0, 0(%r10)
	vmovupd		%ymm4, 32(%r10)
	addq		%r11, %r10
	vmovupd		0(%r10), %ymm15
	vblendpd	$ 0x1, %ymm15, %ymm1, %ymm1
	vmovupd		%ymm1, 0(%r10)
	vmovupd		%ymm5, 32(%r10)
	addq		%r11, %r10
	vmovupd		0(%r10), %ymm15
	vblendpd	$ 0x3, %ymm15, %ymm2, %ymm2
	vmovupd		%ymm2, 0(%r10)
	vmovupd		%ymm6, 32(%r10)
	addq		%r11, %r10
	vmovupd		0(%r10), %ymm15
	vblendpd	$ 0x7, %ymm15, %ymm3, %ymm3
	vmovupd		%ymm3, 0(%r10)
	vmovupd		%ymm7, 32(%r10)
	addq		%r11, %r10

	vmovupd		%ymm8, 32(%r10)
	addq		%r11, %r10
	vmovupd		32(%r10), %ymm15
	vblendpd	$ 0x1, %ymm15, %ymm9, %ymm9
	vmovupd		%ymm9, 32(%r10)
	addq		%r11, %r10
	vmovupd		32(%r10), %ymm15
	vblendpd	$ 0x3, %ymm15, %ymm10, %ymm10
	vmovupd		%ymm10, 32(%r10)
	addq		%r11, %r10
	vmovupd		32(%r10), %ymm15
	vblendpd	$ 0x7, %ymm15, %ymm11, %ymm11
	vmovupd		%ymm11, 32(%r10)
//	addq		%r11, %r10

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_8x8_lib)
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
	.macro INNER_STORE_U_8X8_LIB
#else
	.p2align 4,,15
	FUN_START(inner_store_u_8x8_lib)
#endif

	vmovupd		0(%r10), %ymm15
	vblendpd	$ 0x1, %ymm0, %ymm15, %ymm0
	vmovupd		%ymm0, 0(%r10)
	addq		%r11, %r10
	vmovupd		0(%r10), %ymm15
	vblendpd	$ 0x3, %ymm1, %ymm15, %ymm1
	vmovupd		%ymm1, 0(%r10)
	addq		%r11, %r10
	vmovupd		0(%r10), %ymm15
	vblendpd	$ 0x7, %ymm2, %ymm15, %ymm2
	vmovupd		%ymm2, 0(%r10)
	addq		%r11, %r10
	vmovupd		%ymm3, 0(%r10)
	addq		%r11, %r10

	vmovupd		%ymm4,  0(%r10)
	vmovupd		32(%r10), %ymm15
	vblendpd	$ 0x1, %ymm8, %ymm15, %ymm8
	vmovupd		%ymm8,  32(%r10)
	addq		%r11, %r10
	vmovupd		%ymm5,  0(%r10)
	vmovupd		32(%r10), %ymm15
	vblendpd	$ 0x3, %ymm9, %ymm15, %ymm9
	vmovupd		%ymm9,  32(%r10)
	addq		%r11, %r10
	vmovupd		%ymm6,  0(%r10)
	vmovupd		32(%r10), %ymm15
	vblendpd	$ 0x7, %ymm10, %ymm15, %ymm10
	vmovupd		%ymm10, 32(%r10)
	addq		%r11, %r10
	vmovupd		%ymm7,  0(%r10)
	vmovupd		%ymm11, 32(%r10)
//	addq		%r11, %r10

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_u_8x8_lib)
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
	.macro INNER_STORE_L_8X8_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_store_l_8x8_vs_lib)
#endif
	
	vcvtsi2sd	%r12d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC03(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC03(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm13

	vmovupd		%ymm0, 0(%r10)
	vmaskmovpd	%ymm4, %ymm13, 32(%r10)
	addq		%r11, %r10
//	cmpl		$ 2, %r13d
//	jl			0f // end
	vmovupd		0(%r10), %ymm15
	vblendpd	$ 0x1, %ymm15, %ymm1, %ymm1
	vmovupd		%ymm1, 0(%r10)
	vmaskmovpd	%ymm5, %ymm13, 32(%r10)
	addq		%r11, %r10
//	cmpl		$ 3, %r13d
//	jl			0f // end
	vmovupd		0(%r10), %ymm15
	vblendpd	$ 0x3, %ymm15, %ymm2, %ymm2
	vmovupd		%ymm2, 0(%r10)
	vmaskmovpd	%ymm6, %ymm13, 32(%r10)
	addq		%r11, %r10
//	cmpl		$ 4, %r13d
//	jl			0f // end
	vmovupd		0(%r10), %ymm15
	vblendpd	$ 0x7, %ymm15, %ymm3, %ymm3
	vmovupd		%ymm3, 0(%r10)
	vmaskmovpd	%ymm7, %ymm13, 32(%r10)
	addq	    %r11, %r10
//	cmpl		$ 5, %r13d
//	jl			0f // end

	vmaskmovpd	%ymm8, %ymm13, 32(%r10)
	addq		%r11, %r10
	cmpl		$ 6, %r13d
	jl			0f // end
	vmovupd		32(%r10), %ymm15
	vblendpd	$ 0x1, %ymm15, %ymm9, %ymm9
	vmaskmovpd	%ymm9, %ymm13, 32(%r10)
	addq		%r11, %r10
	cmpl		$ 7, %r13d
	jl			0f // end
	vmovupd		32(%r10), %ymm15
	vblendpd	$ 0x3, %ymm15, %ymm10, %ymm10
	vmaskmovpd	%ymm10, %ymm13, 32(%r10)
	addq		%r11, %r10
	cmpl		$ 7, %r13d
	je			0f // end
	vmovupd		32(%r10), %ymm15
	vblendpd	$ 0x7, %ymm15, %ymm11, %ymm11
	vmaskmovpd	%ymm11, %ymm13, 32(%r10)
//	addq	%r11, %r10

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_8x8_vs_lib)
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
	.macro INNER_STORE_U_8X8_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_store_u_8x8_vs_lib)
#endif
	
	vcvtsi2sd	%r12d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC03(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC03(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm13

	vmovupd		0(%r10), %ymm15
	vblendpd	$ 0x1, %ymm0, %ymm15, %ymm0
	vmovupd		%ymm0, 0(%r10)
	addq		%r11, %r10
//	cmpl		$ 2, %r13d
//	jl			0f // end
	vmovupd		0(%r10), %ymm15
	vblendpd	$ 0x3, %ymm1, %ymm15, %ymm1
	vmovupd		%ymm1, 0(%r10)
	addq		%r11, %r10
//	cmpl		$ 3, %r13d
//	jl			0f // end
	vmovupd		0(%r10), %ymm15
	vblendpd	$ 0x7, %ymm2, %ymm15, %ymm2
	vmovupd		%ymm2, 0(%r10)
	addq		%r11, %r10
//	cmpl		$ 4, %r13d
//	jl			0f // end
	vmovupd		%ymm3, 0(%r10)
	addq		%r11, %r10
//	cmpl		$ 5, %r13d
//	jl			0f // end

	vmovupd		%ymm4,  0(%r10)
	vmovupd		32(%r10), %ymm15
	vblendpd	$ 0x1, %ymm8, %ymm15, %ymm8
	vmaskmovpd	%ymm8,  %ymm13, 32(%r10)
	addq		%r11, %r10
	cmpl		$ 6, %r13d
	jl			0f // end
	vmovupd		%ymm5,  0(%r10)
	vmovupd		32(%r10), %ymm15
	vblendpd	$ 0x3, %ymm9, %ymm15, %ymm9
	vmaskmovpd	%ymm9,  %ymm13, 32(%r10)
	addq		%r11, %r10
	cmpl		$ 7, %r13d
	jl			0f // end
	vmovupd		%ymm6,  0(%r10)
	vmovupd		32(%r10), %ymm15
	vblendpd	$ 0x7, %ymm10, %ymm15, %ymm10
	vmaskmovpd	%ymm10, %ymm13, 32(%r10)
	addq		%r11, %r10
	cmpl		$ 7, %r13d
	je			0f // end
	vmovupd		%ymm7,  0(%r10)
	vmaskmovpd	%ymm11, %ymm13, 32(%r10)
//	addq		%r11, %r10

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_u_8x8_vs_lib)
#endif





//                                   1      2              3          4        5          6        7             8          9        10         11
// void kernel_dsyrk_nt_l_8x8_lib44cc(int k, double *alpha, double *A, int sda, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nt_l_8x8_lib44cc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13 // B
	movq	ARG6, %r14 // sdb
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
#endif


	// call inner blender nn

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12 // C
	movq	ARG9, %r13 // ldc
	sall	$ 3, %r13d // sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8_LIB
#else
	CALL(inner_scale_ab_8x8_lib)
#endif


	// store n

	movq	ARG10, %r10 // store address D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d // sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X8_LIB
#else
	CALL(inner_store_l_8x8_lib)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dsyrk_nt_l_8x8_lib44cc)





//                                       1      2              3          4        5          6        7             8          9        10         11       12      13
// void kernel_dsyrk_nt_l_8x8_vs_lib44cc(int k, double *alpha, double *A, int sda, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nt_l_8x8_vs_lib44cc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13 // B
	movq	ARG6, %r14 // sdb
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
#endif


	// call inner blender nn

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12 // C
	movq	ARG9, %r13 // ldc
	sall	$ 3, %r13d // sdc*sizeof(double)
	movq	ARG12, %r14 // m1
	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8_VS_LIB
#else
	CALL(inner_scale_ab_8x8_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // store address D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d // sdd*sizeof(double)
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X8_VS_LIB
#else
	CALL(inner_store_l_8x8_vs_lib)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dsyrk_nt_l_8x8_vs_lib44cc)





//                                   1      2              3          4        5          6        7             8          9        10         11
// void kernel_dsyrk_nt_u_8x8_lib44cc(int k, double *alpha, double *A, int sda, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nt_u_8x8_lib44cc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG5, %r11 // B
	movq	ARG6, %r12 // sdb
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG3, %r13 // A
	movq	ARG4, %r14 // sda
	sall	$ 5, %r14d // 4*sda*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
#endif


	// call inner blender nn

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12 // C
	movq	ARG9, %r13 // ldc
	sall	$ 3, %r13d // sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_TRAN_SCALE_AB_8X8_LIB
#else
	CALL(inner_tran_scale_ab_8x8_lib)
#endif


	// store n

	movq	ARG10, %r10 // store address D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d // sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_U_8X8_LIB
#else
	CALL(inner_store_u_8x8_lib)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dsyrk_nt_u_8x8_lib44cc)





//                                      1      2              3          4        5          6        7             8          9        10         11       12      13
// void kernel_dsyrk_nt_u_8x8_vs_lib44cc(int k, double *alpha, double *A, int sda, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nt_u_8x8_vs_lib44cc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG5, %r11 // B
	movq	ARG6, %r12 // sdb
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG3, %r13 // A
	movq	ARG4, %r14 // sda
	sall	$ 5, %r14d // 4*sda*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
#endif


	// call inner blender nn

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12 // C
	movq	ARG9, %r13 // ldc
	sall	$ 3, %r13d // sdc*sizeof(double)
	movq	ARG12, %r14 // m1
	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_TRAN_SCALE_AB_8X8_VS_LIB
#else
	CALL(inner_tran_scale_ab_8x8_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // store address D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d // sdd*sizeof(double)
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_U_8X8_VS_LIB
#else
	CALL(inner_store_u_8x8_vs_lib)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dsyrk_nt_u_8x8_vs_lib44cc)





//                                     1      2              3           4         5           6         7           8         9           10        11            12         13       14         15
// void kernel_dsyr2k_nt_l_8x8_lib44cc(int k, double *alpha, double *A0, int sda0, double *B0, int sdb0, double *A1, int sda1, double *B1, int sdb1, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyr2k_nt_l_8x8_lib44cc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A0
	movq	ARG4, %r12 // sda0
	sall	$ 5, %r12d // 4*sda0*sizeof(double)
	movq	ARG5, %r13 // B0
	movq	ARG6, %r14 // sdb0
	sall	$ 5, %r14d // 4*sdb0*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
#endif

	movq	ARG1, %r10 // k
	movq	ARG7, %r11 // A0
	movq	ARG8, %r12 // sda0
	sall	$ 5, %r12d // 4*sda0*sizeof(double)
	movq	ARG9, %r13 // B0
	movq	ARG10, %r14 // sdb0
	sall	$ 5, %r14d // 4*sdb0*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
#endif


	// call inner blender nn

	movq	ARG2, %r10 // alpha
	movq	ARG11, %r11 // beta
	movq	ARG12, %r12 // C
	movq	ARG13, %r13 // ldc
	sall	$ 3, %r13d // sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8_LIB
#else
	CALL(inner_scale_ab_8x8_lib)
#endif


	// store n

	movq	ARG14, %r10 // store address D
	movq	ARG15, %r11 // ldd
	sall	$ 3, %r11d // sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X8_LIB
#else
	CALL(inner_store_l_8x8_lib)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dsyr2k_nt_l_8x8_lib44cc)





//                                        1      2              3           4         5           6         7           8         9           10        11            12         13       14         15       16      17
// void kernel_dsyr2k_nt_l_8x8_vs_lib44cc(int k, double *alpha, double *A0, int sda0, double *B0, int sdb0, double *A1, int sda1, double *B1, int sdb1, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyr2k_nt_l_8x8_vs_lib44cc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A0
	movq	ARG4, %r12 // sda0
	sall	$ 5, %r12d // 4*sda0*sizeof(double)
	movq	ARG5, %r13 // B0
	movq	ARG6, %r14 // sdb0
	sall	$ 5, %r14d // 4*sdb0*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
#endif

	movq	ARG1, %r10 // k
	movq	ARG7, %r11 // A0
	movq	ARG8, %r12 // sda0
	sall	$ 5, %r12d // 4*sda0*sizeof(double)
	movq	ARG9, %r13 // B0
	movq	ARG10, %r14 // sdb0
	sall	$ 5, %r14d // 4*sdb0*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
#endif


	// call inner blender nn

	movq	ARG2, %r10 // alpha
	movq	ARG11, %r11 // beta
	movq	ARG12, %r12 // C
	movq	ARG13, %r13 // ldc
	sall	$ 3, %r13d // sdc*sizeof(double)
	movq	ARG16, %r14 // m1
	movq	ARG17, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8_VS_LIB
#else
	CALL(inner_scale_ab_8x8_vs_lib)
#endif


	// store n

	movq	ARG14, %r10 // store address D
	movq	ARG15, %r11 // ldd
	sall	$ 3, %r11d // sdd*sizeof(double)
	movq	ARG16, %r12 // m1
	movq	ARG17, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X8_VS_LIB
#else
	CALL(inner_store_l_8x8_vs_lib)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dsyr2k_nt_l_8x8_vs_lib44cc)





//                                    1      2          3        4          5        6          7        8          9        10
// void kernel_dpotrf_nt_l_8x8_lib44cc(int k, double *A, int sda, double *B, int sdb, double *C, int ldc, double *D, int ldd, double *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpotrf_nt_l_8x8_lib44cc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // sdb
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
#endif


	// call inner blender nn

	movq	ARG6, %r10 // C
	movq	ARG7, %r11 // ldc
	sall	$ 3, %r11d // sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8X8_LIB
#else
	CALL(inner_scale_m11_8x8_lib)
#endif


	// factorization

	movq	ARG10, %r10  // inv_diag_D 
	movl	$ 8, %r11d

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_8X8_VS_LIB4
#else
	CALL(inner_edge_dpotrf_8x8_vs_lib4)
#endif


	// store n

	movq	ARG8, %r10 // store address D
	movq	ARG9, %r11 // ldd
	sall	$ 3, %r11d // sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X8_LIB
#else
	CALL(inner_store_l_8x8_lib)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dpotrf_nt_l_8x8_lib44cc)





//                                       1      2          3        4          5        6          7        8          9        10                  11      12
// void kernel_dpotrf_nt_l_8x8_vs_lib44cc(int k, double *A, int sda, double *B, int sdb, double *C, int ldc, double *D, int ldd, double *inv_diag_D, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpotrf_nt_l_8x8_vs_lib44cc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // sdb
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
#endif


	// call inner blender nn

	movq	ARG6, %r10 // C
	movq	ARG7, %r11 // ldc
	sall	$ 3, %r11d // sdc*sizeof(double)
	movq	ARG11, %r12 // m1
	movq	ARG12, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8X8_VS_LIB
#else
	CALL(inner_scale_m11_8x8_vs_lib)
#endif


	// factorization

	movq	ARG10, %r10  // inv_diag_D 
	movq	ARG12, %r11

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_8X8_VS_LIB4
#else
	CALL(inner_edge_dpotrf_8x8_vs_lib4)
#endif


	// store n

	movq	ARG8, %r10 // store address D
	movq	ARG9, %r11 // ldd
	sall	$ 3, %r11d // sdd*sizeof(double)
	movq	ARG11, %r12 // m1
	movq	ARG12, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X8_VS_LIB
#else
	CALL(inner_store_l_8x8_vs_lib)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dpotrf_nt_l_8x8_vs_lib44cc)






