function(shell arg1)
  set(cmd ${arg1})
  foreach(arg ${ARGN})
    set(cmd "${cmd} ${arg}")
  endforeach()

  execute_process(COMMAND ${arg1} ${ARGN}
    RESULT_VARIABLE res
    OUTPUT_VARIABLE out
    ERROR_VARIABLE out)

  if(res EQUAL 0)
    debug_message(2 "execute_process(${cmd}) succeeded returning: ${out}")
  else()
    message(FATAL_ERROR "execute_process(${cmd})\n***FAILED with ERROR:***\n${out}")
  endif()
endfunction()
