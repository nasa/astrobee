# This function will prepare a test for a target (specified by the
# variable TEST_TARGET) and execute it. This consists of compiling
# a C file with associated assembly files using the compile flags,
# and then executing the result to see if it runs correctly.
#
# The assembly files contain an exemplar instruction for the ISA,
# so if they fail to run it means the specific ISA is not supported.
#
# The requested target to test is passed as the argument to the function.
#
# The results of the test are stored as the variables
#  CHKISA_TARGET_BUILD - True if the target built without error
#  CHKISA_TARGET_RUN   - True if the test ran without error
function(TestISA TEST_TARGET)

  # Pull in the tests each architecture needs to run
  include( ${PROJECT_SOURCE_DIR}/cmake/ArchitectureTests.cmake )

  # The main source file to test with
  set(CMP_CHECK_SRCS
      ${PROJECT_SOURCE_DIR}/cmake/isa_tests/isa_test.c
      )

  set(C_DEFS_CHK "")

  # Add the assembly test files and the compile definitions
  foreach(CHECK ${CMP_CHECK_${TEST_TARGET}})
      list( APPEND CMP_CHECK_SRCS ${PROJECT_SOURCE_DIR}/cmake/isa_tests/${CHECK}.S )
      list( APPEND C_DEFS_CHK "-D${CHECK} " )
  endforeach()

  string( REPLACE ";" "" C_DEFS_CHK "${C_DEFS_CHK}" )

  # Populate the flags to use for the testing
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${C_FLAGS_TARGET_${TEST_TARGET}}")
  set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${ASM_FLAGS_TARGET_${TEST_TARGET}}")

  if(${BLASFEO_CROSSCOMPILING})
    set(CHKISA_TARGET_RUN_${TEST_TARGET} "1")

    # Only tell CMake to compile the files, not link them since we are doing cross-compilation
    if (${CMAKE_VERSION} VERSION_EQUAL "3.6.0" OR ${CMAKE_VERSION} VERSION_GREATER "3.6")
      set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
    elseif()
      set(CMAKE_EXE_LINKER_FLAGS_INIT "--specs=nosys.specs")
    endif()

    try_compile( CHKISA_TARGET_BUILD_${TEST_TARGET}                   # Variable to save the build result to
                 "${CMAKE_BINARY_DIR}/compilerTest/${TEST_TARGET}" # Directory to compile in
                 SOURCES ${CMP_CHECK_SRCS}                         # Source to compile
                 CMAKE_FLAGS
                   "-DCOMPILE_DEFINITIONS=${C_DEFS_CHK}"
                 OUTPUT_VARIABLE CHK_OUTPUT_${TEST_TARGET}
                )
  else()
    try_run( CHKISA_TARGET_RUN_${TEST_TARGET}                     # Variable to save the run result to
             CHKISA_TARGET_BUILD_${TEST_TARGET}                   # Variable to save the build result to
             "${CMAKE_BINARY_DIR}/compilerTest/${TEST_TARGET}" # Directory to compile in
             SOURCES ${CMP_CHECK_SRCS}                         # Source to compile
             CMAKE_FLAGS
              "-DCOMPILE_DEFINITIONS=${C_DEFS_CHK}"
             OUTPUT_VARIABLE CHK_OUTPUT_${TEST_TARGET}
            )
  endif()

#  message(${CHK_OUTPUT${TEST_TARGET}})

  if(${CHKISA_TARGET_BUILD_${TEST_TARGET}})
    set(CHKISA_TARGET_BUILD TRUE PARENT_SCOPE)

    if(${CHKISA_TARGET_RUN_${TEST_TARGET}} STREQUAL "0")
      set(CHKISA_TARGET_RUN TRUE PARENT_SCOPE)
    else()
      set(CHKISA_TARGET_RUN FALSE PARENT_SCOPE)
    endif()

  else()
    set(CHKISA_TARGET_BUILD FALSE PARENT_SCOPE)
    set(CHKISA_TARGET_OUTPUT ${CHK_OUTPUT_${TEST_TARGET}} PARENT_SCOPE)
  endif()

endfunction()
