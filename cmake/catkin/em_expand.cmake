macro(em_expand context_in context_out em_file_in file_out)
  assert_file_exists("${context_in}" "input file for context missing")
  assert_file_exists("${em_file_in}" "template file missing")
  debug_message(2 "configure_file(${context_in}, ${context_out})")
  configure_file(${context_in} ${context_out} @ONLY)
  assert_file_exists("${context_out}" "context file was not generated correctly")

  stamp(${em_file_in})

  # create directory if necessary
  get_filename_component(_folder_out ${file_out} PATH)
  if(NOT IS_DIRECTORY ${_folder_out})
    file(MAKE_DIRECTORY ${_folder_out})
  endif()

  debug_message(2 "Evaluate template '${em_file_in}' to '${file_out}' (with context from '${context_out}')")
  assert(EMPY_SCRIPT)
  # since empy contains a specific python version in its shebang line
  # override the used python version by invoking it explicitly
  set(command "${PYTHON_EXECUTABLE};${EMPY_SCRIPT}")
  # prepend environment if set
  if(CATKIN_ENV)
    set(command ${CATKIN_ENV} ${command})
  endif()
  safe_execute_process(COMMAND
    ${command}
    --raw-errors
    -F ${context_out}
    -o ${file_out}
    ${em_file_in})
endmacro()
