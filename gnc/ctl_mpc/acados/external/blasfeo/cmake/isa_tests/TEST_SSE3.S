// void test_sse3();

  .p2align 4,,15
#if defined(OS_LINUX)
  .globl test_sse3
  .type test_sse3, @function
test_sse3:
#elif defined(OS_MAC)
  .globl _test_sse3
_test_sse3:
#elif defined(OS_WINDOWS)
  .globl test_sse3
  .def test_sse3; .scl 2; .type 32; .endef
test_sse3:
#endif

  // Call the ADDSUBPD function to see if it works
  // The actual values in the registers don't matter, so just pick two
  addsubpd %xmm1, %xmm0

  ret


#if defined(OS_LINUX)
  .size test_sse3, .-test_sse3
#endif
