include( ${PROJECT_SOURCE_DIR}/cmake/isa_tests/isa_test.cmake )
include( ${PROJECT_SOURCE_DIR}/cmake/intrinsic_tests/intrinsic_test.cmake )

function( X64AutomaticTargetSelection )

  # Iterate over each target to test the compilation and running
  foreach( TEST_TARGET ${X64_AUTOMATIC_TARGETS} )
    # This function will test the compilation and running of the
    # target specified in TEST_TARGET
    TestISA( ${TEST_TARGET} )

    set( ISA_TEST_PASS FALSE )

    if( ${CHKISA_TARGET_BUILD} )
      message( STATUS "Testing target ${TEST_TARGET}: assembly compilation [success]" )

      if( ${CHKISA_TARGET_RUN} )
        message(STATUS "Testing target ${TEST_TARGET}: assembly run [success]" )

        set( ISA_TEST_PASS TRUE )

      else()
        message( STATUS "Testing target ${TEST_TARGET}: assembly run [failed]" )
      endif()

    else()
      message( STATUS "Testing target ${TEST_TARGET}: assembly compilation [failed]" )
    endif()

    TestIntrinsics( ${TEST_TARGET} )

    set( INTRINSIC_TEST_PASS FALSE )

    if( ${CHKINTRINSIC_TARGET_BUILD} )
      message( STATUS "Testing target ${TEST_TARGET}: intrinsic compilation [success]" )

      if( ${CHKINTRINSIC_TARGET_RUN} )
        message( STATUS "Testing target ${TEST_TARGET}: intrinsic run [success]" )

        set( INTRINSIC_TEST_PASS TRUE )

      else()
        message( STATUS "Testing target ${TEST_TARGET}: intrinsic run [failed]" )
      endif()

    else()
      message( STATUS "Testing target ${TEST_TARGET}: intrinsic compilation [failed]" )
    endif()


    if( ${ISA_TEST_PASS} AND ${INTRINSIC_TEST_PASS} )
      # It both compiles and runs, so pass it up to the parent to use
      set( TARGET ${TEST_TARGET} PARENT_SCOPE )
      return()
    endif()

  endforeach()

  message( FATAL_ERROR "Unable to identify a target to use. Please select one manually." )

endfunction()
