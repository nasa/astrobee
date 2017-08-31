macro(safe_execute_process cmd_keyword arg1)
  set(_cmd ${arg1})
  foreach(_arg ${ARGN})
    set(_cmd "${_cmd} \"${_arg}\"")
  endforeach()

  debug_message(2 "execute_process(${_cmd})")
  execute_process(${ARGV} RESULT_VARIABLE _res)

  if(NOT _res EQUAL 0)
    message(FATAL_ERROR "execute_process(${_cmd}) returned error code ${_res}")
  endif()
endmacro()
