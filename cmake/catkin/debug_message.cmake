# Log levels
# 0 Normal use
# 1 Catkin developer use (Stuff being developed)
# 2 Catkin developer use (Stuff working)
# 3 Also Print True Assert Statements

macro(debug_message level)
  set(loglevel ${CATKIN_LOG})
  if(NOT loglevel)
    set(loglevel 0)
  endif()

  if(NOT ${level} GREATER ${loglevel})
    message(STATUS "  ${ARGN}")
  endif()
endmacro()
