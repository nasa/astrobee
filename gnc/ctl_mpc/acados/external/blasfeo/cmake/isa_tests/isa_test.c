
void test_avx();
void test_avx2();
void test_fma();
void test_sse3();
void test_vfpv4();
void test_neonv2();
void test_vfpv3();
void test_neon();


int main()
{

#ifdef TEST_AVX
    // Test for if the AVX ISA works
    test_avx();
#endif

#ifdef TEST_AVX2
    // Test for if the AVX2 ISA works
    test_avx2();
#endif

#ifdef TEST_FMA
    // Test for if the FMA ISA works
    test_fma();
#endif

#ifdef TEST_SSE3
    // Test for if the SSE3 ISA works
    test_sse3();
#endif

#ifdef TEST_VFPv4
    // Test for if the VFPv4 ISA works
    test_vfpv4();
#endif

#ifdef TEST_NEONv2
    // Test for if the NEONv2 ISA works
    test_neonv2();
#endif

#ifdef TEST_VFPv3
    // Test for if the VFPv3 ISA works
    test_vfpv3();
#endif

#ifdef TEST_NEON
    // Test for if the NEON ISA works
    test_neon();
#endif

    return 0;
}
