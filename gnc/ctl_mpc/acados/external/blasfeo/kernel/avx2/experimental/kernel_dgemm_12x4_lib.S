/* experimental stuff !!!!!!!! */



// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B

//
// output arguments:
// r10d  <- 0
// r11   <- A+4*k*sizeof(double)
// r12   <- B+4*k*sizeof(double)

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_12X4_LIBA4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_12x4_liba4)
#endif
	
// broadcast scheme

	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch
	prefetcht0	0(%r12) // software prefetch
	prefetcht0	0+64(%r12) // software prefetch

	// preload
	vmovapd 0(%r11), %ymm13 // A0[0]
	vmovapd 32(%r11), %ymm14 // A1[0]
	vmovapd 64(%r11), %ymm15 // A1[0]

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	prefetcht0	128(%r12) // software prefetch
	prefetcht0	128+64(%r12) // software prefetch

//	prefetcht0	384(%r11) // software prefetch
//	prefetcht0	384+1*64(%r11) // software prefetch
//	prefetcht0	384+2*64(%r11) // software prefetch
//	prefetcht0	384+3*64(%r11) // software prefetch
//	prefetcht0	384+4*64(%r11) // software prefetch
//	prefetcht0	384+5*64(%r11) // software prefetch

	// unroll 0
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	8(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	subl	$ 4, %r10d

	vbroadcastsd	16(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	24(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			96(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			128(%r11), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			160(%r11), %ymm15 // A1

	// unroll 1
	vbroadcastsd	32(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	40(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	48(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	56(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			192(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			224(%r11), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			256(%r11), %ymm15 // A1

	// unroll 2
	vbroadcastsd	64(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	72(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	80(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	88(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			288(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			320(%r11), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			352(%r11), %ymm15 // A1

	// unroll 3
	vbroadcastsd	96(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	addq	$ 384, %r11

	vbroadcastsd	104(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	112(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	120(%r12), %ymm12
	addq	$ 128, %r12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			0(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			32(%r11), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			64(%r11), %ymm15 // A1


	cmpl	$ 4, %r10d
	jg		1b // main loop 

0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	8(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	subl	$ 4, %r10d

	vbroadcastsd	16(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	24(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			96(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			128(%r11), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			160(%r11), %ymm15 // A1

	// unroll 1
	vbroadcastsd	32(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	40(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	48(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	56(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			192(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			224(%r11), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			256(%r11), %ymm15 // A1

	// unroll 2
	vbroadcastsd	64(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	72(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	80(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	88(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			288(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			320(%r11), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			352(%r11), %ymm15 // A1

	// unroll 3
	vbroadcastsd	96(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	addq	$ 384, %r11

	vbroadcastsd	104(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	112(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	120(%r12), %ymm12
	addq	$ 128, %r12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
//	vmovapd			0(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
//	vmovapd			32(%r11), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
//	vmovapd			64(%r11), %ymm15 // A1



	jmp		2f


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	vmovapd			0(%r11), %ymm13 // A0[0]
	vmovapd 		32(%r11), %ymm14 // A1[0]
	vmovapd 		64(%r11), %ymm15 // A2[0]
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	8(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	addq	$ 96, %r11

	vbroadcastsd	16(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	subl	$ 1, %r10d

	vbroadcastsd	24(%r12), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	$ 32, %r12

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

FUN_END(inner_kernel_dgemm_nt_12x4_liba4)
#endif





//                                   1      2              3          4        5          6             7          8        9
// void kernel_dgemm_nt_12x4_liba4cc(int k, double *alpha, double *A, double *B, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_12x4_liba4cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// prefetch C

	movq		ARG5, %r10 // beta
	vmovsd		0(%r10), %xmm14
	vxorpd		%xmm15, %xmm15, %xmm15 // 0.0
	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			100f // end

	movq	ARG6, %r10 // C
	movq	ARG7, %r11 // ldc
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH_12X4_LIB
#else
	CALL(inner_prefetch_12x4_lib)
#endif

100:


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIBA4
#else
	CALL(inner_kernel_dgemm_nt_12x4_liba4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C
	movq	ARG7, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB
#else
	CALL(inner_scale_ab_12x4_lib)
#endif


	// store n

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

	FUN_END(kernel_dgemm_nt_12x4_liba4cc)





//                                     1      2              3          4        5          6             7          8        9          10       11
// void kernel_dgemm_nt_12x4_vs_liba4cc(int k, double *alpha, double *A, double *B, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_12x4_vs_liba4cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIBA4
#else
	CALL(inner_kernel_dgemm_nt_12x4_liba4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C
	movq	ARG7, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG10, %r14 // ldd
	movq	ARG11, %r15 // ldd

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_VS_LIB
#else
	CALL(inner_scale_ab_12x4_vs_lib)
#endif


	// store n

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG10, %r12 // ldd
	movq	ARG11, %r13 // ldd

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB
#else
	CALL(inner_store_12x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_12x4_vs_liba4cc)







