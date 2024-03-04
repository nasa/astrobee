// void test_avx2();

  .p2align 4,,15
#if defined(OS_LINUX)
  .globl test_avx2
  .type test_avx2, @function
test_avx2:
#elif defined(OS_MAC)
  .globl _test_avx2
_test_avx2:
#elif defined(OS_WINDOWS)
  .globl test_avx2
  .def test_avx2; .scl 2; .type 32; .endef
test_avx2:
#endif

  // Call the VPSRAVD function to see if it works
  // The actual values being used don't matter, just call it
  vpsravd %xmm2, %xmm1, %xmm0

  ret


#if defined(OS_LINUX)
  .size test_avx2, .-test_avx2
#endif
