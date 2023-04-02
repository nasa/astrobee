
#if defined( TEST_AVX ) || defined( TEST_AVX2 ) || defined( TEST_FMA )
    // Header that contains the AVX, AVX2 and FMA intrinsics
    #include <immintrin.h>
#endif

#ifdef TEST_SSE3
    // Header that contains the SSE3 intrinsics
    #include <pmmintrin.h>
#endif


int main()
{

#ifdef TEST_AVX
    // Test for working AVX intrinsics

    // This setter is AVX minimum
    __m256d retVal_AVX = _mm256_set_pd( 1.0f, 2.0f, 3.0f, 4.0f );
#endif

#ifdef TEST_AVX2
    // Test for working AVX2 intrinsics

    // This setter is AVX minimum
    __m256i testVal_AVX2_1 = _mm256_set_epi32( 1, 2, 3, 4, 5, 6, 7, 8 );
    __m256i testVal_AVX2_2 = _mm256_set_epi32( 2, 3, 4, 5, 6, 7, 8, 9 );

    // This subtraction is AVX2 minimum
    __m256i retVal_AVX2 = _mm256_sub_epi32( testVal_AVX2_1, testVal_AVX2_2 );
#endif

#ifdef TEST_FMA
    // Test for working FMA intrinsics

    // These brodcast setters are SSE minimum
    __m128 testVal_FMA_1 = _mm_set1_ps( 5.0 );
    __m128 testVal_FMA_2 = _mm_set1_ps( 3.0 );
    __m128 testVal_FMA_3 = _mm_set1_ps( 7.0 );

    // This is FMA minimum
    __m128 retVal_FMA = _mm_fmadd_ps( testVal_FMA_1, testVal_FMA_2, testVal_FMA_3 );
#endif

#ifdef TEST_SSE3
    // Test for working SSE3 intrinsics

    // These brodcast setters are SSE minimum
    __m128 testVal_SSE3_1 = _mm_set1_ps( 5.0 );
    __m128 testVal_SSE3_2 = _mm_set1_ps( 3.0 );

    // This adder is SSE3 minimum
    __m128 retVal_SSE3 = _mm_hadd_ps( testVal_SSE3_1, testVal_SSE3_2 );
#endif

#ifdef TEST_VFPv4
    // Test for working VFPv4 intrinsics

    //TODO
#endif

#ifdef TEST_NEONv2
    // Test for working NEONv2 intrinsics

    //TODO
#endif

#ifdef TEST_VFPv3
    // Test for working VFPv3 intrinsics

    // TODO
#endif

#ifdef TEST_NEON
    // Test for working NEON intrinsics


    //TODO
#endif

    return 0;
}
