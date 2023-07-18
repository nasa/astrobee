/* experimental stuff !!! */



#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACK_TT_4_LIB12
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpack_tt_4_lib12)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 11, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	movq	%r11, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 0(%r13)
	vmovupd		32(%r15), %ymm0
	vmovapd		%ymm0, 32(%r13)
	vmovupd		64(%r15), %ymm0
	vmovapd		%ymm0, 64(%r13)
	addq	%r12, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 96(%r13)
	vmovupd		32(%r15), %ymm0
	vmovapd		%ymm0, 128(%r13)
	vmovupd		64(%r15), %ymm0
	vmovapd		%ymm0, 160(%r13)
	addq	%r12, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 192(%r13)
	vmovupd		32(%r15), %ymm0
	vmovapd		%ymm0, 224(%r13)
	vmovupd		64(%r15), %ymm0
	vmovapd		%ymm0, 256(%r13)
	addq	%r12, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 288(%r13)
	vmovupd		32(%r15), %ymm0
	vmovapd		%ymm0, 320(%r13)
	vmovupd		64(%r15), %ymm0
	vmovapd		%ymm0, 352(%r13)
	addq	%r12, %r15

	subl	$ 12, %r10d
	addq	$ 96, %r11
	addq	%r14, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	movq	%r11, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 0(%r13)
	addq	%r12, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 32(%r13)
	addq	%r12, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 64(%r13)
	addq	%r12, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 96(%r13)
	addq	%r12, %r15

	subl	$ 1, %r10d
	addq	$ 8, %r11
	addq	$ 8, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpack_tt_4_lib12)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACK_TT_3_LIB12
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpack_tt_3_lib12)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 11, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	movq	%r11, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 0(%r13)
	vmovupd		32(%r15), %ymm0
	vmovapd		%ymm0, 32(%r13)
	vmovupd		64(%r15), %ymm0
	vmovapd		%ymm0, 64(%r13)
	addq	%r12, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 96(%r13)
	vmovupd		32(%r15), %ymm0
	vmovapd		%ymm0, 128(%r13)
	vmovupd		64(%r15), %ymm0
	vmovapd		%ymm0, 160(%r13)
	addq	%r12, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 192(%r13)
	vmovupd		32(%r15), %ymm0
	vmovapd		%ymm0, 224(%r13)
	vmovupd		64(%r15), %ymm0
	vmovapd		%ymm0, 256(%r13)
	addq	%r12, %r15

//	vmovupd		0(%r15), %ymm0
//	vmovapd		%ymm0, 288(%r13)
//	vmovupd		32(%r15), %ymm0
//	vmovapd		%ymm0, 320(%r13)
//	vmovupd		64(%r15), %ymm0
//	vmovapd		%ymm0, 352(%r13)
//	addq	%r12, %r15

	subl	$ 12, %r10d
	addq	$ 96, %r11
	addq	%r14, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	movq	%r11, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 0(%r13)
	addq	%r12, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 32(%r13)
	addq	%r12, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 64(%r13)
	addq	%r12, %r15

//	vmovsd		0(%r15), %xmm0
//	vmovsd		%xmm0, 96(%r13)
//	addq	%r12, %r15

	subl	$ 1, %r10d
	addq	$ 8, %r11
	addq	$ 8, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpack_tt_3_lib12)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACK_TT_2_LIB12
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpack_tt_2_lib12)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 11, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	movq	%r11, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 0(%r13)
	vmovupd		32(%r15), %ymm0
	vmovapd		%ymm0, 32(%r13)
	vmovupd		64(%r15), %ymm0
	vmovapd		%ymm0, 64(%r13)
	addq	%r12, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 96(%r13)
	vmovupd		32(%r15), %ymm0
	vmovapd		%ymm0, 128(%r13)
	vmovupd		64(%r15), %ymm0
	vmovapd		%ymm0, 160(%r13)
	addq	%r12, %r15

//	vmovupd		0(%r15), %ymm0
//	vmovapd		%ymm0, 192(%r13)
//	vmovupd		32(%r15), %ymm0
//	vmovapd		%ymm0, 224(%r13)
//	vmovupd		64(%r15), %ymm0
//	vmovapd		%ymm0, 256(%r13)
//	addq	%r12, %r15

//	vmovupd		0(%r15), %ymm0
//	vmovapd		%ymm0, 288(%r13)
//	vmovupd		32(%r15), %ymm0
//	vmovapd		%ymm0, 320(%r13)
//	vmovupd		64(%r15), %ymm0
//	vmovapd		%ymm0, 352(%r13)
//	addq	%r12, %r15

	subl	$ 12, %r10d
	addq	$ 96, %r11
	addq	%r14, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	movq	%r11, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 0(%r13)
	addq	%r12, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 32(%r13)
	addq	%r12, %r15

//	vmovsd		0(%r15), %xmm0
//	vmovsd		%xmm0, 64(%r13)
//	addq	%r12, %r15

//	vmovsd		0(%r15), %xmm0
//	vmovsd		%xmm0, 96(%r13)
//	addq	%r12, %r15

	subl	$ 1, %r10d
	addq	$ 8, %r11
	addq	$ 8, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpack_tt_2_lib12)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACK_TT_1_LIB12
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpack_tt_1_lib12)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 11, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	movq	%r11, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 0(%r13)
	vmovupd		32(%r15), %ymm0
	vmovapd		%ymm0, 32(%r13)
	vmovupd		64(%r15), %ymm0
	vmovapd		%ymm0, 64(%r13)
	addq	%r12, %r15

//	vmovupd		0(%r15), %ymm0
//	vmovapd		%ymm0, 96(%r13)
//	vmovupd		32(%r15), %ymm0
//	vmovapd		%ymm0, 128(%r13)
//	vmovupd		64(%r15), %ymm0
//	vmovapd		%ymm0, 160(%r13)
//	addq	%r12, %r15

//	vmovupd		0(%r15), %ymm0
//	vmovapd		%ymm0, 192(%r13)
//	vmovupd		32(%r15), %ymm0
//	vmovapd		%ymm0, 224(%r13)
//	vmovupd		64(%r15), %ymm0
//	vmovapd		%ymm0, 256(%r13)
//	addq	%r12, %r15

//	vmovupd		0(%r15), %ymm0
//	vmovapd		%ymm0, 288(%r13)
//	vmovupd		32(%r15), %ymm0
//	vmovapd		%ymm0, 320(%r13)
//	vmovupd		64(%r15), %ymm0
//	vmovapd		%ymm0, 352(%r13)
//	addq	%r12, %r15

	subl	$ 12, %r10d
	addq	$ 96, %r11
	addq	%r14, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	movq	%r11, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 0(%r13)
	addq	%r12, %r15

//	vmovsd		0(%r15), %xmm0
//	vmovsd		%xmm0, 32(%r13)
//	addq	%r12, %r15

//	vmovsd		0(%r15), %xmm0
//	vmovsd		%xmm0, 64(%r13)
//	addq	%r12, %r15

//	vmovsd		0(%r15), %xmm0
//	vmovsd		%xmm0, 96(%r13)
//	addq	%r12, %r15

	subl	$ 1, %r10d
	addq	$ 8, %r11
	addq	$ 8, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpack_tt_1_lib12)
#endif





//                              1         2          3        4          5
// void kernel_dpack_tt_4_lib12(int kmax, double *A, int lda, double *C, int sdc)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpack_tt_4_lib12)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 3, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // sdc
	sall	$ 5, %r14d
	leal	0(%r14d, %r14d, 2), %r14d


#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACK_TT_4_LIB12
#else
	CALL(inner_kernel_dpack_tt_4_lib12)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dpack_tt_4_lib12)





//                                 1         2          3        4          5        6
// void kernel_dpack_tt_4_vs_lib12(int kmax, double *A, int lda, double *C, int sdc, int m1)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpack_tt_4_vs_lib12)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 3, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // sdc
	sall	$ 5, %r14d
	leal	0(%r14d, %r14d, 2), %r14d

	movq	ARG6, %r15 // m1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACK_TT_1_LIB12
#else
	CALL(inner_kernel_dpack_tt_1_lib12)
#endif

	jmp		103f

100:

	movq	ARG6, %r15 // m1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACK_TT_2_LIB12
#else
	CALL(inner_kernel_dpack_tt_2_lib12)
#endif
	
	jmp		103f

101:

	movq	ARG6, %r15 // m1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACK_TT_3_LIB12
#else
	CALL(inner_kernel_dpack_tt_3_lib12)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACK_TT_4_LIB12
#else
	CALL(inner_kernel_dpack_tt_4_lib12)
#endif

103:


	EPILOGUE

	ret

	FUN_END(kernel_dpack_tt_4_vs_lib12)






