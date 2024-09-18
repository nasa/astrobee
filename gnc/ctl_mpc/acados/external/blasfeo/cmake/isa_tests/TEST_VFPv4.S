// void test_vfpv4()

//  .p2align 4,,15
#if defined(OS_LINUX)
  .global test_vfpv4
  .type test_vfpv4, %function
test_vfpv4:
#elif defined(OS_MAC)
  .global _test_vfpv4
_test_vfpv4:
#endif


// Test for the presence of the fused multiply accumulate instruction
fmadd d0, d1, d2, d3


#if defined(OS_LINUX)
  .size test_vfpv4, .-test_vfpv4
#endif
