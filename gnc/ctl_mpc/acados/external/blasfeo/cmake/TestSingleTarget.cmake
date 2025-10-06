include( ${PROJECT_SOURCE_DIR}/cmake/isa_tests/isa_test.cmake )
include( ${PROJECT_SOURCE_DIR}/cmake/intrinsic_tests/intrinsic_test.cmake )

function( TestSingleTarget )
  set( TEST_TARGET ${TARGET} )

  # This function will test the compilation and running of the
  # target specified in TEST_TARGET
  TestISA( ${TEST_TARGET} )

  if( ${CHKISA_TARGET_BUILD} )
    message( STATUS "Testing target ${TEST_TARGET}: assembly compilation [success]" )

    if( NOT ${BLASFEO_CROSSCOMPILING} )
      if( ${CHKISA_TARGET_RUN})
        message( STATUS "Testing target ${TEST_TARGET}: assembly run [success]" )
      else()
        message( STATUS "Testing target ${TEST_TARGET}: assembly run [failed]" )
      endif()
    endif()

  else()
    message( STATUS "Testing target ${TEST_TARGET}: assembly compilation [failed]" )
    message( "Compile output:" )
    message( ${CHKISA_TARGET_OUTPUT} )
    message( FATAL_ERROR "Unable to compile with assembly for target ${TEST_TARGET}" )
  endif()

  # This function will test the compiler support for intrinsics
  TestIntrinsics( ${TEST_TARGET} )

  if( ${CHKINTRINSIC_TARGET_BUILD} )
    message( STATUS "Testing target ${TEST_TARGET}: intrinsic compilation [success]" )

    if( NOT ${BLASFEO_CROSSCOMPILING} )
      if( ${CHKINTRINSIC_TARGET_RUN} )
        message( STATUS "Testing target ${TEST_TARGET}: intrinsic run [success]" )
      else()
        message( STATUS "Testing target ${TEST_TARGET}: intrinsic run [failed]" )
      endif()
    endif()

  else()
    message( STATUS "Testing target ${TEST_TARGET}: intrinsic compilation [failed]" )
    message( "Compile output:" )
    message( ${CHKINTRINSIC_TARGET_OUTPUT} )
    message( FATAL_ERROR "Unable to compile with intrinsics for target ${TEST_TARGET}" )
  endif()

endfunction()
