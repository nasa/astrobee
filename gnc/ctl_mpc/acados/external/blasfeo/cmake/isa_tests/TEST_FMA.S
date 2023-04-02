// void test_fma();

  .p2align 4,,15
#if defined(OS_LINUX)
  .globl test_fma
  .type test_fma, @function
test_fma:
#elif defined(OS_MAC)
  .globl _test_fma
_test_fma:
#elif defined(OS_WINDOWS)
  .globl test_fma
  .def test_fma; .scl 2; .type 32; .endef
test_fma:
#endif

  // Call the VFMADD231PD function to see if it works
  // The actual values in the registers don't matter, so just pick three
  vfmadd231pd %xmm2, %xmm1, %xmm0

  ret


#if defined(OS_LINUX)
  .size test_fma, .-test_fma
#endif
