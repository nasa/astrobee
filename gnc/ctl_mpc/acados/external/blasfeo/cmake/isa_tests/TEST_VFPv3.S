// void test_vfpv3()

//  .p2align 4,,15
#if defined(OS_LINUX)
  .global test_vfpv3
  .type test_vfpv3, %function
test_vfpv3:
#elif defined(OS_MAC)
  .global _test_vfpv3
_test_vfpv3:
#endif


// Test for the presence of the move constant instruction
// This is the equivalent of the UAL vmov.f32 instruction
fconsts s1, #112 // 1.0


#if defined(OS_LINUX)
  .size test_vfpv3, .-test_vfpv3
#endif
