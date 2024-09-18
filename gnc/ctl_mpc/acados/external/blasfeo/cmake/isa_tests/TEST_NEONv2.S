// void test_neonv2()

//  .p2align 4,,15
#if defined(OS_LINUX)
  .global test_neonv2
  .type test_neonv2, %function
test_neonv2:
#elif defined(OS_MAC)
  .global _test_neonv2
_test_neonv2:
#endif


// Test for the presence of the vector fused multiply accumulate instruction
fmla  v0.2d, v24.2d, v30.2d


#if defined(OS_LINUX)
  .size test_neonv2, .-test_neonv2
#endif
