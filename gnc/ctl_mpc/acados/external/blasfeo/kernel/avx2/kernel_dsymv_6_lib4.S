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

#if defined(OS_LINUX) | defined(OS_MAC)

//#define STACKSIZE 96
#define STACKSIZE 64
#define ARG1  %rdi
#define ARG2  %rsi
#define ARG3  %rdx
#define ARG4  %rcx
#define ARG5  %r8
#define ARG6  %r9
#define ARG7  STACKSIZE +  8(%rsp)
#define ARG8  STACKSIZE + 16(%rsp)
#define ARG9  STACKSIZE + 24(%rsp)
#define ARG10 STACKSIZE + 32(%rsp)
#define ARG11 STACKSIZE + 40(%rsp)
#define ARG12 STACKSIZE + 48(%rsp)
#define ARG13 STACKSIZE + 56(%rsp)
#define ARG14 STACKSIZE + 64(%rsp)
#define ARG15 STACKSIZE + 72(%rsp)
#define ARG16 STACKSIZE + 80(%rsp)
#define ARG17 STACKSIZE + 88(%rsp)
#define ARG18 STACKSIZE + 96(%rsp)
#define PROLOGUE \
	subq	$STACKSIZE, %rsp; \
	movq	%rbx,   (%rsp); \
	movq	%rbp,  8(%rsp); \
	movq	%r12, 16(%rsp); \
	movq	%r13, 24(%rsp); \
	movq	%r14, 32(%rsp); \
	movq	%r15, 40(%rsp); \
	vzeroupper;
#define EPILOGUE \
	vzeroupper; \
	movq	  (%rsp), %rbx; \
	movq	 8(%rsp), %rbp; \
	movq	16(%rsp), %r12; \
	movq	24(%rsp), %r13; \
	movq	32(%rsp), %r14; \
	movq	40(%rsp), %r15; \
	addq	$STACKSIZE, %rsp;

#if defined(OS_LINUX)

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

#else // defined(OS_MAC)

#define GLOB_FUN_START(NAME) \
	.globl _ ## NAME; \
_ ## NAME:
#define FUN_START(NAME) \
_ ## NAME:
#define FUN_END(NAME)
#define CALL(NAME) \
	callq _ ## NAME
//#define ZERO_ACC \
//	vxorpd	%ymm0, %ymm0, %ymm0; \
//	vmovapd	%ymm0, %ymm1; \
//	vmovapd	%ymm0, %ymm2; \
//	vmovapd	%ymm0, %ymm3
//#define NEG_ACC \
//	vmovapd		LC11(%rip), %ymm15; \
//	vxorpd		%ymm15, %ymm0, %ymm0; \
//	vxorpd		%ymm15, %ymm1, %ymm1; \
//	vxorpd		%ymm15, %ymm2, %ymm2; \
//	vxorpd		%ymm15, %ymm3, %ymm3

#endif

#elif defined(OS_WINDOWS)

#define STACKSIZE 256
#define ARG1  %rcx
#define ARG2  %rdx
#define ARG3  %r8
#define ARG4  %r9
#define ARG5  STACKSIZE + 40(%rsp)
#define ARG6  STACKSIZE + 48(%rsp)
#define ARG7  STACKSIZE + 56(%rsp)
#define ARG8  STACKSIZE + 64(%rsp)
#define ARG9  STACKSIZE + 72(%rsp)
#define ARG10 STACKSIZE + 80(%rsp)
#define ARG11 STACKSIZE + 88(%rsp)
#define ARG12 STACKSIZE + 96(%rsp)
#define ARG13 STACKSIZE + 104(%rsp)
#define ARG14 STACKSIZE + 112(%rsp)
#define ARG15 STACKSIZE + 120(%rsp)
#define ARG16 STACKSIZE + 128(%rsp)
#define ARG17 STACKSIZE + 136(%rsp)
#define ARG18 STACKSIZE + 144(%rsp)
#define PROLOGUE \
	subq	$STACKSIZE, %rsp; \
	movq	%rbx,   (%rsp); \
	movq	%rbp,  8(%rsp); \
	movq	%r12, 16(%rsp); \
	movq	%r13, 24(%rsp); \
	movq	%r14, 32(%rsp); \
	movq	%r15, 40(%rsp); \
	movq	%rdi, 48(%rsp); \
	movq	%rsi, 56(%rsp); \
	vmovups	%xmm6, 64(%rsp); \
	vmovups	%xmm7, 80(%rsp); \
	vmovups	%xmm8, 96(%rsp); \
	vmovups	%xmm9, 112(%rsp); \
	vmovups	%xmm10, 128(%rsp); \
	vmovups	%xmm11, 144(%rsp); \
	vmovups	%xmm12, 160(%rsp); \
	vmovups	%xmm13, 176(%rsp); \
	vmovups	%xmm14, 192(%rsp); \
	vmovups	%xmm15, 208(%rsp); \
	vzeroupper;
#define EPILOGUE \
	vzeroupper; \
	movq	  (%rsp), %rbx; \
	movq	 8(%rsp), %rbp; \
	movq	16(%rsp), %r12; \
	movq	24(%rsp), %r13; \
	movq	32(%rsp), %r14; \
	movq	40(%rsp), %r15; \
	movq	48(%rsp), %rdi; \
	movq	56(%rsp), %rsi; \
	vmovups	64(%rsp), %xmm6; \
	vmovups	80(%rsp), %xmm7; \
	vmovups	96(%rsp), %xmm8; \
	vmovups	112(%rsp), %xmm9; \
	vmovups	128(%rsp), %xmm10; \
	vmovups	144(%rsp), %xmm11; \
	vmovups	160(%rsp), %xmm12; \
	vmovups	176(%rsp), %xmm13; \
	vmovups	192(%rsp), %xmm14; \
	vmovups	208(%rsp), %xmm15; \
	addq	$STACKSIZE, %rsp;

#define GLOB_FUN_START(NAME) \
	.globl NAME; \
	.def NAME; .scl 2; .type 32; .endef; \
NAME:
#define FUN_START(NAME) \
	.def NAME; .scl 2; .type 32; .endef; \
NAME:
#define FUN_END(NAME)
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



#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.text
#elif defined(OS_MAC)
	.section	__TEXT,__text,regular,pure_instructions
#endif

// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- bs*sda*sizeof(double) = 32*sda
// r13   <- x_t
// r14   <- z_n
// ymm0  <- [z_t_0a z_t_0b z_t_0c z_t_0d]
// ymm1  <- [z_t_1a z_t_1b z_t_1c z_t_1d]
// ymm2  <- [z_t_2a z_t_2b z_t_2c z_t_2d]
// ymm3  <- [z_t_3a z_t_3b z_t_3c z_t_3d]
// ymm4  <- [z_t_4a z_t_4b z_t_4c z_t_4d]
// ymm5  <- [z_t_5a z_t_5b z_t_5c z_t_5d]
// ymm6  <- x_n_0
// ymm7  <- x_n_1
// ymm8  <- x_n_2
// ymm9  <- x_n_3
// ymm10 <- x_n_4
// ymm11 <- x_n_5
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

//
// output arguments:
// r10d  <- 0
// r11   <- A+4*k*sizeof(double)
// r12   <- bs*sda*sizeof(double) = 32*sda
// r13   <- x_t+k*sizeof(double)
// r14   <- z_n+k*sizeof(double)
// ymm0  <- [z_t_0a z_t_0b z_t_0c z_t_0d]
// ymm1  <- [z_t_1a z_t_1b z_t_1c z_t_1d]
// ymm2  <- [z_t_2a z_t_2b z_t_2c z_t_2d]
// ymm3  <- [z_t_3a z_t_3b z_t_3c z_t_3d]
// ymm4  <- [z_t_4a z_t_4b z_t_4c z_t_4d]
// ymm5  <- [z_t_5a z_t_5b z_t_5c z_t_5d]
// ymm6  <- x_n_0
// ymm7  <- x_n_1
// ymm8  <- x_n_2
// ymm9  <- x_n_3
// ymm10 <- x_n_4
// ymm11 <- x_n_5
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMV_ADD_NT_6_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemv_add_nt_6_lib4)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	cmpl	$ 4, %r10d
	jl		0f // clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	vmovupd	0(%r13), %ymm12
	vmovupd	0(%r14), %ymm13

	vmovapd	0(%r11), %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm0
	vfmadd231pd	%ymm14, %ymm6, %ymm13
	
	subl	$ 4, %r10d

	vmovapd	32(%r11), %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm1
	vfmadd231pd	%ymm14, %ymm7, %ymm13
	
	vmovapd	64(%r11), %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm2
	vfmadd231pd	%ymm14, %ymm8, %ymm13

	vmovapd	96(%r11), %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm3
	vfmadd231pd	%ymm14, %ymm9, %ymm13
	
	vmovapd	128(%r11), %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm4
	vfmadd231pd	%ymm14, %ymm10, %ymm13
	
	vmovapd	160(%r11), %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm5
	vfmadd231pd	%ymm14, %ymm11, %ymm13
	
	vmovupd	%ymm13, 0(%r14) 

	addq	%r12, %r11
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
	vsubpd		%ymm14, %ymm13, %ymm15

	vmaskmovpd	0(%r13), %ymm15, %ymm12
	vmaskmovpd	0(%r14), %ymm15, %ymm13

	vmaskmovpd	0(%r11), %ymm15, %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm0
	vfmadd231pd	%ymm14, %ymm6, %ymm13
	
	vmaskmovpd	32(%r11), %ymm15, %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm1
	vfmadd231pd	%ymm14, %ymm7, %ymm13
	
	vmaskmovpd	64(%r11), %ymm15, %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm2
	vfmadd231pd	%ymm14, %ymm8, %ymm13

	vmaskmovpd	96(%r11), %ymm15, %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm3
	vfmadd231pd	%ymm14, %ymm9, %ymm13
	
	vmaskmovpd	128(%r11), %ymm15, %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm4
	vfmadd231pd	%ymm14, %ymm10, %ymm13
	
	vmaskmovpd	160(%r11), %ymm15, %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm5
	vfmadd231pd	%ymm14, %ymm11, %ymm13
	
	vmaskmovpd	%ymm13, %ymm15, 0(%r14)

	sall	$ 3, %r10d
	addq	%r10, %r11
	addq	%r10, %r13
	addq	%r10, %r14
	xorl	%r10d, %r10d
	
	
2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemv_add_nt_6_lib4)
#endif






#if 0

// TODO
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
// ymm10 <- dirty
// ymm11 <- dirty
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

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
// ymm10 <- dirty
// ymm11 <- dirty
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=2
	.macro INNER_EDGE_DSYMV_ADD_NT_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dsymv_add_nt_4_lib4)
#endif

	vmovupd		0(%r13), %ymm12
	vmovupd		0(%r14), %ymm13

	vmovapd		0(%r11), %ymm14
	vmulpd		%ymm14, %ymm12, %ymm15
	vaddpd		%ymm0, %ymm15, %ymm0
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x1, %ymm15, %ymm14, %ymm14
	vmulpd		%ymm14, %ymm6, %ymm15
	vaddpd		%ymm13, %ymm15, %ymm13
	
	vmovapd		32(%r11), %ymm14
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x1, %ymm15, %ymm14, %ymm14
	vmulpd		%ymm14, %ymm12, %ymm15
	vaddpd		%ymm1, %ymm15, %ymm1
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x3, %ymm15, %ymm14, %ymm14
	vmulpd		%ymm14, %ymm7, %ymm15
	vaddpd		%ymm13, %ymm15, %ymm13
	
	vmovapd		64(%r11), %ymm14
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x3, %ymm15, %ymm14, %ymm14
	vmulpd		%ymm14, %ymm12, %ymm15
	vaddpd		%ymm2, %ymm15, %ymm2
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x7, %ymm15, %ymm14, %ymm14
	vmulpd		%ymm14, %ymm8, %ymm15
	vaddpd		%ymm13, %ymm15, %ymm13

	vmovapd		96(%r11), %ymm14
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x7, %ymm15, %ymm14, %ymm14
	vmulpd		%ymm14, %ymm12, %ymm15
	vaddpd		%ymm3, %ymm15, %ymm3
//	vxorpd		%ymm15, %ymm15, %ymm15
//	vblendpd	$ 0x0, %ymm14, %ymm15, %ymm14
//	vmulpd		%ymm14, %ymm9, %ymm15
//	vaddpd		%ymm13, %ymm15, %ymm13
	
	vmovupd		%ymm13, 0(%r14) 

	addq	%r12, %r11
	addq	$ 32, %r13
	addq	$ 32, %r14
	
	subq	$ 4, %r10

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_edge_dsymv_add_nt_4_lib4)
#endif

#endif





// common inner routine with file scope
//
// blend for ta==t, scale for generic alpha and beta
//
// input arguments:
// r10  <- alpha
// r11  <- beta
// r12  <- y
// ymm0 <- [z0a z0b z0c z0d]
// ymm1 <- [z1a z1b z1c z1d]
// ymm2 <- [z2a z2b z2c z2d]
// ymm3 <- [z3a z3b z3c z3d]
// ymm4 <- [z4a z4b z4c z4d]
// ymm5 <- [z5a z5b z5c z5d]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- alpha
// r11  <- beta
// r12  <- y
// ymm0 <- [z0 z1 z2 z3]
// ymm1 <- [z4 z5 xx xx]
// ymm1 <- dirty
// ymm2 <- dirty
// ymm3 <- dirty
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_T_SCALE_AB_6_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_t_scale_ab_6_lib4)
#endif

	// reduction
	vhaddpd			%ymm1, %ymm0, %ymm0
	vhaddpd			%ymm3, %ymm2, %ymm2
	vhaddpd			%ymm5, %ymm4, %ymm4
//	vhaddpd			%ymm3, %ymm2, %ymm2
	vperm2f128		$ 0x2, %ymm0, %ymm2, %ymm1
	vperm2f128		$ 0x13, %ymm0, %ymm2, %ymm0
	vextractf128	$ 0x1, %ymm4, %xmm5
	vaddpd			%ymm0, %ymm1, %ymm0
	vaddpd			%ymm4, %ymm5, %ymm4

	// alpha
	vbroadcastsd	0(%r10), %ymm15
	vmulpd			%ymm0, %ymm15, %ymm0
	vmulpd			%ymm4, %ymm15, %ymm1

	// beta
	vbroadcastsd	0(%r11), %ymm15
	vmovupd			0(%r12), %ymm14
	vmovupd			32(%r12), %ymm13
	vfmadd231pd		%ymm15, %ymm14, %ymm0
	vfmadd231pd		%ymm15, %ymm13, %ymm1
	
	vxorpd			%ymm15, %ymm15, %ymm15
	vblendpd		$ 0x3, %ymm1, %ymm15, %ymm1

#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_blend_t_scale_ab_6_lib4)
#endif





#if 0

//TODO
// common inner routine with file scope
//
// blend for ta==t, scale for generic alpha and beta=1.0
//
// input arguments:
// r10  <- alpha
// r11  <- y
// ymm0 <- [z0a z0b z0c z0d]
// ymm1 <- [z1a z1b z1c z1d]
// ymm2 <- [z2a z2b z2c z2d]
// ymm3 <- [z3a z3b z3c z3d]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- alpha
// r11  <- y
// ymm0 <- [z0 z1 z2 z3]
// ymm1 <- dirty
// ymm2 <- dirty
// ymm3 <- dirty
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_T_SCALE_A1_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_t_scale_a1_4_lib4)
#endif

	// reduction
	vhaddpd	%ymm1, %ymm0, %ymm0
	vhaddpd	%ymm3, %ymm2, %ymm2
	vperm2f128	$ 0x2, %ymm0, %ymm2, %ymm1
	vperm2f128	$ 0x13, %ymm0, %ymm2, %ymm0
	vaddpd	%ymm0, %ymm1, %ymm0

	// alpha
	vbroadcastsd	0(%r10), %ymm15
	vmulpd	%ymm0, %ymm15, %ymm0

	// beta
	vmovupd		0(%r11), %ymm14
	vaddpd		%ymm0, %ymm14, %ymm0
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_blend_t_scale_a1_4_lib4)
#endif

#endif




// common inner routine with file scope
//
// store 
//
// input arguments:
// r10  <- z
// ymm0 <- [z0 z1 z2 z3]
//
// output arguments:
// r10  <- z
// ymm0 <- [z0 z1 z2 z3]

#if MACRO_LEVEL>=1
	.macro INNER_STORE_6_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_6_lib4)
#endif
	
	vmovupd %ymm0, 0(%r10)
	vmovupd %xmm1, 32(%r10)
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_6_lib4)
#endif





//                             rdi    rsi              rdx              rcx        r8       r9           rsp+8        rsp+16          rsp+24       rsp_32       rsp_40
// void kernel_dgemv_nt_6_lib4(int k, double *alpha_n, double *alpha_t, double *A, int sda, double *x_n, double *x_t, double *beta_t, double *y_t, double *z_n, double *z_t);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemv_nt_6_lib4)
	
	PROLOGUE

	// zero accumulation registers y_t

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3
	vmovapd	%ymm0, %ymm4
	vmovapd	%ymm0, %ymm5

	// initialize x_n
	movq	ARG2, %r10 // alpha_n
	vbroadcastsd 0(%r10), %ymm15

	movq	ARG6, %r10 // x_n

	vbroadcastsd 0(%r10), %ymm6
	vmulpd		%ymm15, %ymm6, %ymm6
	vbroadcastsd 8(%r10), %ymm7
	vmulpd		%ymm15, %ymm7, %ymm7
	vbroadcastsd 16(%r10), %ymm8
	vmulpd		%ymm15, %ymm8, %ymm8
	vbroadcastsd 24(%r10), %ymm9
	vmulpd		%ymm15, %ymm9, %ymm9
	vbroadcastsd 32(%r10), %ymm10
	vmulpd		%ymm15, %ymm10, %ymm10
	vbroadcastsd 40(%r10), %ymm11
	vmulpd		%ymm15, %ymm11, %ymm11


	// inner kernel dgemv nt

	movq	ARG1, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
//	movslq	%r12d, %r12
	movq	ARG7, %r13  // x_t
	movq	ARG10, %r14  // z_n

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMV_ADD_NT_6_LIB4
#else
	CALL(inner_kernel_dgemv_add_nt_6_lib4)
#endif


	// inner blend n scale ab

	movq	ARG3, %r10 // alpha_t
	movq	ARG8, %r11   // beta_t
	movq	ARG9, %r12   // y_t

#if MACRO_LEVEL>=1
	INNER_BLEND_T_SCALE_AB_6_LIB4
#else
	CALL(inner_blend_t_scale_ab_6_lib4)
#endif


	// store

	movq	ARG11, %r10 // z_t 

#if MACRO_LEVEL>=1
	INNER_STORE_6_LIB4
#else
	CALL(inner_store_6_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemv_nt_6_lib4)





#if 0
// TODO
//                            rdi    rsi            rdx        rcx      r8           r9           rsp+8        rsp+16 
// void kernel_dsymv_l_4_lib4(int k, double *alpha, double *A, int sda, double *x_n, double *x_t, double *z_n, double *z_t);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsymv_l_4_lib4)
	
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
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
//	movslq	%r12d, %r12
	movq	ARG6, %r13  // x_t
	movq	ARG7, %r14  // z_n

#if MACRO_LEVEL>=2
	INNER_EDGE_DSYMV_ADD_NT_4_LIB4
#else
	CALL(inner_edge_dsymv_add_nt_4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMV_ADD_NT_4_LIB4
#else
	CALL(inner_kernel_dgemv_add_nt_4_lib4)
#endif


	// call inner blend n scale ab

	movq	ARG2, %r10 // alpha
	movq	ARG8, %r11   // z_t

#if MACRO_LEVEL>=1
	INNER_BLEND_T_SCALE_A1_4_LIB4
#else
	CALL(inner_blend_t_scale_a1_4_lib4)
#endif


	// store

	movq	ARG8, %r10 // z_t 

#if MACRO_LEVEL>=1
	INNER_STORE_4_LIB4
#else
	CALL(inner_store_4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dsymv_l_4_lib4)

#endif





	// read-only data
#if defined(OS_LINUX)
	.section	.rodata.cst32,"aM",@progbits,32
#elif defined(OS_MAC)
	.section	__TEXT,__const
#elif defined(OS_WINDOWS)
	.section .rdata,"dr"
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC00: // { -1 -1 -1 1 }
#elif defined(OS_MAC)
LC00: // { -1 -1 -1 1 }
	.align 5
#endif
	.quad	-1
	.quad	-1
	.quad	-1
	.quad	1

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC01: // { -1 -1 -1 -1 }
#elif defined(OS_MAC)
LC01: // { -1 -1 -1 -1 }
	.align 5
#endif
	.quad	-1
	.quad	-1
	.quad	-1
	.quad	-1

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC02: // { 3.5 2.5 1.5 0.5 }
#elif defined(OS_MAC)
LC02: // { 3.5 2.5 1.5 0.5 }
	.align 5
#endif
	.long	0
	.long	1071644672
	.long	0
	.long	1073217536
	.long	0
	.long	1074003968
	.long	0
	.long	1074528256

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC03: // { 7.5 6.5 5.5 4.5 }
#elif defined(OS_MAC)
LC03: // { 7.5 6.5 5.5 4.5 }
	.align 5
#endif
	.long	0
	.long	1074921472
	.long	0
	.long	1075183616
	.long	0
	.long	1075445760
	.long	0
	.long	1075707904

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC04: // { 1.0 1.0 1.0 1.0 }
#elif defined(OS_MAC)
LC04: // { 1.0 1.0 1.0 1.0 }
	.align 5
#endif
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248



#if defined(OS_LINUX)
	.section	.note.GNU-stack,"",@progbits
#elif defined(OS_MAC)
	.subsections_via_symbols
#endif





