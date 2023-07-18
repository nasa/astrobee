# This file contains the various architecture tests that each target
# must pass when testing the ISA and intrinsics.

# For Intel Haswell, test for if the AVX2 and FMA ISAs work
set(CMP_CHECK_X64_INTEL_HASWELL
  TEST_AVX2
  TEST_FMA
  )

# For Intel Sandy Bridge, test for if the AVX ISA works
set(CMP_CHECK_X64_INTEL_SANDY_BRIDGE
  TEST_AVX
  )

# For Intel Core, test for if the SSE3 ISA works
set(CMP_CHECK_X64_INTEL_CORE
  TEST_SSE3
  )

# For AMD Bulldozer, test for if the AVX and FMA ISAs work
set(CMP_CHECK_X64_AMD_BULLDOZER
  TEST_AVX
  TEST_FMA
  )

# For the Cortex A57, test for if the VFPv4 and NEONv2 ISAs work
set(CMP_CHECK_ARMV8A_ARM_CORTEX_A57
  TEST_VFPv4
  TEST_NEONv2
  )

# For the Cortex A53, test for if the VFPv4 and NEONv2 ISAs work
set(CMP_CHECK_ARMV8A_ARM_CORTEX_A53
  TEST_VFPv4
  TEST_NEONv2
  )

# For the Cortex A15, test for if the VFPv3 and NEON ISAs work
set(CMP_CHECK_ARMV7A_ARM_CORTEX_A15
  TEST_VFPv3
  TEST_NEON
  )

# For the Cortex A7, test for if the VFPv3 and NEON ISAs work
set(CMP_CHECK_ARMV7A_ARM_CORTEX_A7
  TEST_VFPv3
  TEST_NEON
  )

# For the Cortex A9, test for if the VFPv3 and NEON ISAs work
set(CMP_CHECK_ARMV7A_ARM_CORTEX_A9
  TEST_VFPv3
  TEST_NEON
  )
