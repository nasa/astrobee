// void test_avx();

  .p2align 4,,15
#if defined(OS_LINUX)
  .globl test_avx
  .type test_avx, @function
test_avx:
#elif defined(OS_MAC)
  .globl _test_avx
_test_avx:
#elif defined(OS_WINDOWS)
  .globl test_avx
  .def test_avx; .scl 2; .type 32; .endef
test_avx2:
#endif

  // Call the VZEROALL function to see if it works
  vzeroall

  ret


#if defined(OS_LINUX)
  .size test_avx, .-test_avx
#endif
